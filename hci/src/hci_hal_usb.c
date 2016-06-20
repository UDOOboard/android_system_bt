/******************************************************************************
 *
 *  Copyright (C) 2016 Seco, Inc.
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at:
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 ******************************************************************************/

#define LOG_TAG "bt_hci_usb"

#include <assert.h>
#include <errno.h>
#include <string.h>


#include "buffer_allocator.h"
#include "bt_vendor_lib.h"

#include "osi/include/eager_reader.h"
#include "hci_hal.h"
#include "osi/include/osi.h"
#include "osi/include/log.h"
#include "osi/include/thread.h"
#include <sys/prctl.h>
#include "libusb/libusb.h"
#include "vendor.h"

/******************************************************************************
**  Constants & Macros
******************************************************************************/

// #define DUMPDATA

#ifndef USB_DBG
#define USB_DBG FALSE
#endif

#if (USB_DBG == TRUE)
#define USBDBG ALOGD
#define USBERR ALOGE
#else
#define USBDBG
#define USBERR
#endif

/*
 * Bit masks : To check the transfer status
 */
#define XMITTED                 1
#define RX_DEAD                 2
#define RX_FAILED               4
#define XMIT_FAILED             8

/*
 * Field index values
 */
#define EV_LEN_FIELD        1
#define BLK_LEN_LO          2
#define BLK_LEN_HI          3
#define SCO_LEN_FIELD       2

#define BT_CTRL_EP      0x0
#define BT_INT_EP       0x81
#define BT_BULK_IN      0x82
#define BT_BULK_OUT     0x02
#define BT_ISO_IN       0x83
#define BT_ISO_OUT      0x03

#define INT_CMD_PKT_IDX_MASK        0x07

#define BT_HCI_MAX_FRAME_SIZE      1028

#define BT_MAX_ISO_FRAMES   10

#define H4_TYPE_COMMAND         1
#define H4_TYPE_ACL_DATA        2
#define H4_TYPE_SCO_DATA        3
#define H4_TYPE_EVENT           4

static const uint16_t msg_evt_table[] =
{
    MSG_HC_TO_STACK_HCI_ERR,       /* H4_TYPE_COMMAND */
    MSG_HC_TO_STACK_HCI_ACL,       /* H4_TYPE_ACL_DATA */
    MSG_HC_TO_STACK_HCI_SCO,       /* H4_TYPE_SCO_DATA */
    MSG_HC_TO_STACK_HCI_EVT        /* H4_TYPE_EVENT */
};

/* Num of allowed outstanding HCI CMD packets */
volatile int num_hci_cmd_pkts = 1;

#define ACL_RX_PKT_START        2
#define ACL_RX_PKT_CONTINUE     1
#define L2CAP_HEADER_SIZE       4

#define HCI_COMMAND_COMPLETE_EVT    0x0E
#define HCI_COMMAND_STATUS_EVT      0x0F
#define HCI_READ_BUFFER_SIZE        0x1005
#define HCI_LE_READ_BUFFER_SIZE     0x2002

/*
 * USB types, the second of three bRequestType fields
 */
#define USB_TYPE_REQ                 32

/* Preamble length for HCI Commands:
**      2-bytes for opcode and 1 byte for length
*/
#define HCI_CMD_PREAMBLE_SIZE   3

/* Preamble length for HCI Events:
**      1-byte for opcode and 1 byte for length
*/
#define HCI_EVT_PREAMBLE_SIZE   2

/* Preamble length for SCO Data:
**      2-byte for Handle and 1 byte for length
*/
#define HCI_SCO_PREAMBLE_SIZE   3

/* Preamble length for ACL Data:
**      2-byte for Handle and 2 byte for length
*/
#define HCI_ACL_PREAMBLE_SIZE   4

/* Table of HCI preamble sizes for the different HCI message types */
static const uint8_t hci_preamble_table[] =
{
    HCI_CMD_PREAMBLE_SIZE,
    HCI_ACL_PREAMBLE_SIZE,
    HCI_SCO_PREAMBLE_SIZE,
    HCI_EVT_PREAMBLE_SIZE
};

#define RX_NEW_PKT              1
#define RECEIVING_PKT           2

#define CONTAINER_RX_HDR(ptr) \
      (RX_HDR *)((char *)(ptr) - offsetof(RX_HDR, data))

#define CONTAINER_ISO_HDR(ptr) \
      (ISO_HDR *)((char *)(ptr) - offsetof(ISO_HDR, data))

#define CONTAINER_CMD_HDR(ptr) \
      (CMD_HDR *)((char *)(ptr) - offsetof(CMD_HDR, data))

/******************************************************************************
**  Local type definitions
******************************************************************************/
/*
The mutex is protecting send_rx_event and rxed_xfer.

rxed_xfer     : Accounting the packet received at recv_xfer_cb() and processed
                at usb_read().
send_rx_event : usb_read() signals recv_xfer_cb() to signal  the
                Host/Controller lib thread about new packet arrival.

usb_read() belongs to Host/Controller lib thread.
recv_xfer_cb() belongs to USB read thread
*/

/* Generic purpose transac returned upon request complete */
typedef void* TRANSAC;

/** Host/Controller Library Return Status */
typedef enum {
    BT_HC_STATUS_SUCCESS,
    BT_HC_STATUS_FAIL,
    BT_HC_STATUS_NOT_READY,
    BT_HC_STATUS_NOMEM,
    BT_HC_STATUS_BUSY,
    BT_HC_STATUS_CORRUPTED_BUFFER
} bt_hc_status_t;

/******************************************************************************
**  Static variables
******************************************************************************/

void inbound_data_waiting(void *context);

int thread_wait = 0;
//int read_in_use = 2;

static pthread_mutex_t utils_mutex;

#define BT_USB_DEVICE_INFO(cl, sc, pr) \
        .bDevClass = (cl), \
        .bDevSubClass = (sc), \
        .bDevProtocol = (pr)


struct bt_usb_device {
    uint8_t    bDevClass;
    uint8_t    bDevSubClass;
    uint8_t    bDevProtocol;
};

typedef struct
{
    void        *p_first;
    void        *p_last;
    uint16_t    count;
} BUFFER_Q;

typedef struct
{
    libusb_device_handle      *handle;
    pthread_t                 read_thread;
    pthread_mutex_t           mutex;
    pthread_cond_t            cond;
    int                       rxed_xfer;
    uint8_t                   send_rx_event;
    BUFFER_Q                  rx_eventq;
    BUFFER_Q                  rx_bulkq;
    BUFFER_Q                  rx_isoq;
    int16_t                   rx_pkt_len;
    uint8_t                   rx_status;
    int                       iso_frame_ndx;
    struct libusb_transfer    *failed_tx_xfer;
} tUSB_CB;

#define INT_CMD_PKT_MAX_COUNT       8

typedef struct
{
    uint16_t opcode;        /* OPCODE of outstanding internal commands */
    tINT_CMD_CBACK cback;   /* Callback function when return of internal
                             * command is received */
} tINT_CMD_Q;

/* H4 Rx States */
typedef enum {
    H4_RX_MSGTYPE_ST,
    H4_RX_LEN_ST,
    H4_RX_DATA_ST,
    H4_RX_IGNORE_ST
} tHCI_H4_RCV_STATE;

typedef struct
{
    uint16_t          event;
    uint16_t          len;
    uint16_t          offset;
    uint16_t          layer_specific;
    uint8_t           data[];
} HC_BT_HDR;

/* Control block for HCISU_H4 */
typedef struct
{
    HC_BT_HDR *p_rcv_msg;          /* Buffer to hold current rx HCI message */
    uint16_t rcv_len;               /* Size of current incoming message */
    uint8_t rcv_msg_type;           /* Current incoming message type */
    tHCI_H4_RCV_STATE rcv_state;    /* Receive state of current rx message */
    uint16_t hc_acl_data_size;      /* Controller's max ACL data length */
    uint16_t hc_ble_acl_data_size;  /* Controller's max BLE ACL data length */
    BUFFER_Q acl_rx_q;      /* Queue of base buffers for fragmented ACL pkts */
    uint8_t preload_count;          /* Count numbers of preload bytes */
    uint8_t preload_buffer[6];      /* HCI_ACL_PREAMBLE_SIZE + 2 */
    int int_cmd_rsp_pending;        /* Num of internal cmds pending for ack */
    uint8_t int_cmd_rd_idx;         /* Read index of int_cmd_opcode queue */
    uint8_t int_cmd_wrt_idx;        /* Write index of int_cmd_opcode queue */
    tINT_CMD_Q int_cmd[INT_CMD_PKT_MAX_COUNT]; /* FIFO queue */
} tHCI_H4_CB;

/******************************************************************************
**  Static variables
******************************************************************************/
/* The list will grow and will be updated from btusb.c in kernel */
static struct bt_usb_device btusb_table[] = {
    /* Generic Bluetooth USB device */
    { BT_USB_DEVICE_INFO(0xe0, 0x01, 0x01) },
    { BT_USB_DEVICE_INFO(0xef, 0x02, 0x01) },
    { }     /* Terminating entry */
};

typedef struct
{
    uint16_t          event;
    uint16_t          len;
    uint16_t          offset;
    unsigned char     data[0];
} RX_HDR;

struct iso_frames
{
    int               actual_length;
    int               length;
};

typedef struct
{
    uint16_t           event;
    uint16_t           len;
    uint16_t           offset;
    struct iso_frames  frames[BT_MAX_ISO_FRAMES];
    unsigned char      data[0];
} ISO_HDR;

typedef struct
{
    uint8_t                     event;
    struct libusb_control_setup setup;
    unsigned char               data[0];
} CMD_HDR;

/******************************************************************************
**  Type definitions and return values
******************************************************************************/

#define BT_HC_HDR_SIZE (sizeof(HC_BT_HDR))

typedef struct _hc_buffer_hdr
{
    struct _hc_buffer_hdr *p_next;   /* next buffer in the queue */
    uint8_t   reserved1;
    uint8_t   reserved2;
    uint8_t   reserved3;
    uint8_t   reserved4;
} HC_BUFFER_HDR_T;

static tHCI_H4_CB       h4_cb;

#define BT_HC_BUFFER_HDR_SIZE (sizeof(HC_BUFFER_HDR_T))

/******************************************************************************
**  Type definitions
******************************************************************************/

/** Prototypes for HCI Service interface functions **/

/* Initialize transport's control block */
typedef void (*tHCI_INIT)(void);

/* Do transport's control block clean-up */
typedef void (*tHCI_CLEANUP)(void);

/* Send HCI command/data to the transport */
typedef void (*tHCI_SEND)(HC_BT_HDR *p_msg);

/* Handler for HCI upstream path */
typedef uint16_t (*tHCI_RCV)(void);

/* Callback function for the returned event of internally issued command */
typedef void (*tINT_CMD_CBACK)(void *p_mem);

/* Handler for sending HCI command from the local module */
typedef uint8_t (*tHCI_SEND_INT)(uint16_t opcode, HC_BT_HDR *p_buf, \
                                  tINT_CMD_CBACK p_cback);

/* Handler for getting acl data length */
typedef void (*tHCI_ACL_DATA_LEN_HDLR)(void);

typedef struct {
    tHCI_INIT init;
    tHCI_CLEANUP cleanup;
    tHCI_SEND send;
    tHCI_SEND_INT send_int_cmd;
    tHCI_ACL_DATA_LEN_HDLR get_acl_max_len;
    tHCI_RCV rcv;
} tHCI_IF;

//tHCI_IF *p_hci_if;
static tUSB_CB usb;
// extern int num_hci_cmd_pkts;
static bool tx_cmd_pkts_pending = false;
static int usb_xfer_status, usb_running;
static int intr_pkt_size, iso_pkt_size, bulk_pkt_size;
static int intr_pkt_size_wh, iso_pkt_size_wh, bulk_pkt_size_wh;
static struct libusb_transfer *data_rx_xfer, *event_rx_xfer, *iso_rx_xfer,
                              *xmit_transfer;
// bt_hc_callbacks_t *bt_hc_cbacks = NULL;
static const allocator_t *bt_hc_cbacks;
// static const allocator_t *buffer_allocator;

// bt_hc_cbacks = buffer_allocator_get_interface();
static int xmited_len;
RX_HDR *p_rx_hdr = NULL;
// Our interface and modules we import
static const hci_hal_t interface;
static const hci_hal_callbacks_t *callbacks;
//static const vendor_t *vendor;

static thread_t *thread; // Not owned by us

//static eager_reader_t *usb_stream;
static serial_data_type_t current_data_type;
static bool stream_has_interpretation;
static bool stream_corruption_detected;
static uint8_t stream_corruption_bytes_to_ignore;

static void event_usb_has_bytes(void);

static void hal_close();
void utils_queue_init (BUFFER_Q *p_q);

static bool hal_init(const hci_hal_callbacks_t *upper_callbacks, thread_t *upper_thread) {
  assert(upper_callbacks != NULL);
  assert(upper_thread != NULL);

    memset(&usb, 0, sizeof(tUSB_CB));
    usb.handle = NULL;
    utils_queue_init(&(usb.rx_eventq));
    utils_queue_init(&(usb.rx_bulkq));
    utils_queue_init(&(usb.rx_isoq));
    pthread_mutex_init(&usb.mutex, NULL);
    pthread_cond_init(&usb.cond, NULL);
    data_rx_xfer = event_rx_xfer = iso_rx_xfer = NULL;
    usb.send_rx_event = TRUE;
    usb.rx_status = RX_NEW_PKT;

   callbacks = upper_callbacks;
   thread = upper_thread;

  return true;
}

uint16_t  usb_read(uint16_t msg_id, uint8_t *p_buffer, uint16_t len);

static void event_rx(UNUSED_ATTR void *context) {

    uint8_t     byte;

    while (TRUE)
    {
        /* Read one byte to see if there is anything waiting to be read */
        if (usb_read(0 /*dummy*/, &byte, 1) == 1)
        {
    	  stream_has_interpretation = true;
    	  current_data_type = byte;
          break;
        }
    }
    event_usb_has_bytes();

   if (tx_cmd_pkts_pending && num_hci_cmd_pkts > 0) {
     // Got HCI Cmd credits from controller. Send whatever data
     // we have in our tx queue. We can call |event_tx| directly
     // here since we're already on the worker thread.
     event_tx(NULL);
   }
}

/******************************************************************************
**  Local type definitions
******************************************************************************/

/* Host/Controller lib thread control block */
typedef struct
{
    thread_t        *worker_thread;
    pthread_mutex_t worker_thread_lock;
    bool            epilog_timer_created;
    timer_t         epilog_timer_id;
} bt_hc_cb_t;

/******************************************************************************
**  Static Variables
******************************************************************************/

static bt_hc_cb_t hc_cb;
static BUFFER_Q tx_q;

/*******************************************************************************
**
** Function        utils_queue_init
**
** Description     Initialize the given buffer queue
**
** Returns         None
**
*******************************************************************************/
void utils_queue_init (BUFFER_Q *p_q)
{
    p_q->p_first = p_q->p_last = NULL;
    p_q->count = 0;
}

/*******************************************************************************
**
** Function        utils_enqueue
**
** Description     Enqueue a buffer at the tail of the given queue
**
** Returns         None
**
*******************************************************************************/
void utils_enqueue (BUFFER_Q *p_q, void *p_buf)
{
    HC_BUFFER_HDR_T    *p_hdr;

    p_hdr = (HC_BUFFER_HDR_T *) ((uint8_t *) p_buf - BT_HC_BUFFER_HDR_SIZE);

    pthread_mutex_lock(&utils_mutex);

    if (p_q->p_last)
    {
        HC_BUFFER_HDR_T *p_last_hdr = \
          (HC_BUFFER_HDR_T *)((uint8_t *)p_q->p_last - BT_HC_BUFFER_HDR_SIZE);

        p_last_hdr->p_next = p_hdr;
    }
    else
        p_q->p_first = p_buf;

    p_q->p_last = p_buf;
    p_q->count++;

    p_hdr->p_next = NULL;

    pthread_mutex_unlock(&utils_mutex);
}

/*******************************************************************************
**
** Function        utils_dequeue_unlocked
**
** Description     Dequeues a buffer from the head of the given queue without lock
**
** Returns         NULL if queue is empty, else buffer
**
*******************************************************************************/
void *utils_dequeue_unlocked (BUFFER_Q *p_q)
{
    HC_BUFFER_HDR_T    *p_hdr;


    if (!p_q || !p_q->count)
    {
        return (NULL);
    }

    p_hdr=(HC_BUFFER_HDR_T *)((uint8_t *)p_q->p_first-BT_HC_BUFFER_HDR_SIZE);

    if (p_hdr->p_next)
        p_q->p_first = ((uint8_t *)p_hdr->p_next + BT_HC_BUFFER_HDR_SIZE);
    else
    {
        p_q->p_first = NULL;
        p_q->p_last  = NULL;
    }

    p_q->count--;

    p_hdr->p_next = NULL;
    return ((uint8_t *)p_hdr + BT_HC_BUFFER_HDR_SIZE);
}
/*******************************************************************************
**
** Function        utils_dequeue
**
** Description     Dequeues a buffer from the head of the given queue
**
** Returns         NULL if queue is empty, else buffer
**
*******************************************************************************/
void *utils_dequeue (BUFFER_Q *p_q)
{
    pthread_mutex_lock(&utils_mutex);
    void* p_buf =  utils_dequeue_unlocked(p_q);
    pthread_mutex_unlock(&utils_mutex);
    return p_buf;
}


/*******************************************************************************
**
** Function        utils_delay
**
** Description     sleep unconditionally for timeout milliseconds
**
** Returns         None
**
*******************************************************************************/
void utils_delay (uint32_t timeout)
{
    struct timespec delay;
    int err;

    delay.tv_sec = timeout / 1000;
    delay.tv_nsec = 1000 * 1000 * (timeout%1000);

    /* [u]sleep can't be used because it uses SIGALRM */
    do {
        err = nanosleep(&delay, &delay);
    } while (err < 0 && errno ==EINTR);
}

void bthc_rx_ready(void) {
  pthread_mutex_lock(&hc_cb.worker_thread_lock);

  if (hc_cb.worker_thread) {
    thread_post(hc_cb.worker_thread, event_rx, NULL);
  }

  pthread_mutex_unlock(&hc_cb.worker_thread_lock);
}


static void usb_rx_signal_event()
{
    pthread_mutex_lock(&usb.mutex);
    usb.rxed_xfer++;
    pthread_cond_signal(&usb.cond);

    if (usb.send_rx_event == TRUE)
    {
       bthc_rx_ready();
       usb.send_rx_event = FALSE;
    }
    pthread_mutex_unlock(&usb.mutex);
}

static void recv_xfer_cb(struct libusb_transfer *transfer)
{
    RX_HDR *p_rx = NULL;
    ISO_HDR *p_iso = NULL;
    int r, i, offset = 0, len = 0, skip = 0;
    enum libusb_transfer_status status;


    status = transfer->status;
    if (status == LIBUSB_TRANSFER_COMPLETED)
    {
        switch (transfer->endpoint)
        {
            struct iso_frames *frames;
            case BT_INT_EP:
                if (transfer->actual_length == 0)
                {
                    USBDBG("*****Rxed zero length packet from usb ....");
                    skip = 1;
                    break;
                }
                p_rx = CONTAINER_RX_HDR(transfer->buffer);
                p_rx->event = H4_TYPE_EVENT;
                p_rx->len = (uint16_t)transfer->actual_length;
                utils_enqueue(&(usb.rx_eventq), p_rx);
                p_rx =  (RX_HDR *) bt_hc_cbacks->alloc(intr_pkt_size_wh);
                transfer->buffer = p_rx->data;
                transfer->length = intr_pkt_size;
                break;

            case BT_BULK_IN:
                if (transfer->actual_length == 0)
                {
                    USBDBG("*******Rxed zero length packet from usb ....");
                    skip = 1;
                    break;
                }
                p_rx = CONTAINER_RX_HDR(transfer->buffer);
                p_rx->event = H4_TYPE_ACL_DATA;
                p_rx->len = (uint16_t)transfer->actual_length;
                utils_enqueue(&(usb.rx_bulkq), p_rx);
                p_rx =  (RX_HDR *) bt_hc_cbacks->alloc(bulk_pkt_size_wh);
                transfer->buffer = p_rx->data;
                transfer->length = bulk_pkt_size;
                break;

            case BT_ISO_IN:
                if (transfer->actual_length == 0)
                {
                    USBDBG("*******Rxed zero length packet from usb ....");
                    skip = 1;
                    break;
                }
                p_iso = CONTAINER_ISO_HDR(transfer->buffer);
                frames = p_iso->frames;
                memset(frames, 0, sizeof(struct iso_frames) * BT_MAX_ISO_FRAMES);
                p_iso->event = H4_TYPE_SCO_DATA;
                for(i = 0; i < transfer->num_iso_packets; i++, frames++)
                {
                    frames->length = transfer->iso_packet_desc[i].length;
                    frames->actual_length = \
                          transfer->iso_packet_desc[i].actual_length;
                    len += frames->actual_length;
                }
                p_iso->len = (uint16_t)len;
                utils_enqueue(&(usb.rx_isoq), p_iso);
                p_iso =  (ISO_HDR *) bt_hc_cbacks->alloc(iso_pkt_size_wh);
                transfer->buffer = p_iso->data;
                for(i = 0; i < BT_MAX_ISO_FRAMES; i++)
                {
                    transfer->iso_packet_desc[i].length = iso_pkt_size;
                }
                break;

            default:
                USBERR("Unexpeted endpoint rx %d\n", transfer->endpoint);
                return;
        }
        if (!skip)
            usb_rx_signal_event();
    }
    else
    {
        USBERR("******Transfer to BT Device failed- %d *****", status);
        usb_xfer_status |= RX_DEAD;
        return;
    }
    r = libusb_submit_transfer(transfer);
    if (r < 0)
    {
        if (transfer->endpoint == BT_ISO_IN)
        {
            p_rx = (RX_HDR *)CONTAINER_ISO_HDR(transfer->buffer);
        }
        else
        {
            p_rx = CONTAINER_RX_HDR(transfer->buffer);
        }
        bt_hc_cbacks->free((TRANSAC)p_rx);
        transfer->buffer = NULL;
        USBERR("libusb_submit_transfer : %d : %d : failed", \
               transfer->endpoint, transfer->status);
        usb_xfer_status |= RX_FAILED;
    }
}


/******************************************************************************
**  Static functions
******************************************************************************/
static int is_usb_match_idtable (struct bt_usb_device *id, struct libusb_device_descriptor *desc)
{
    int ret = TRUE;

    ret = ((id->bDevClass != libusb_le16_to_cpu(desc->bDeviceClass)) ? FALSE :
           (id->bDevSubClass != libusb_le16_to_cpu(desc->bDeviceSubClass)) ? FALSE :
           (id->bDevProtocol != libusb_le16_to_cpu(desc->bDeviceProtocol)) ? FALSE : TRUE);

    return ret;
}

static int check_bt_usb_endpoints (struct bt_usb_device *id, struct libusb_config_descriptor *cfg_desc)
{
    const struct libusb_interface_descriptor *idesc;
    const struct libusb_endpoint_descriptor *endpoint;
    int i, num_altsetting;

    endpoint =  cfg_desc->interface[0].altsetting[0].endpoint;
    for(i = 0; i < cfg_desc->interface[0].altsetting[0].bNumEndpoints; i++)
    {
        if(!(endpoint[i].bEndpointAddress == BT_CTRL_EP || \
              endpoint[i].bEndpointAddress == BT_INT_EP || \
              endpoint[i].bEndpointAddress == BT_BULK_IN || \
              endpoint[i].bEndpointAddress == BT_BULK_OUT))
            return FALSE;
    }

    num_altsetting =  cfg_desc->interface[1].num_altsetting;
    endpoint =  cfg_desc->interface[1].altsetting[num_altsetting-1].endpoint;
    for(i = 0; i < cfg_desc->interface[1].altsetting[num_altsetting-1]. \
          bNumEndpoints; i++)
    {
        if(!(endpoint[i].bEndpointAddress == BT_ISO_IN || \
              endpoint[i].bEndpointAddress == BT_ISO_OUT))
            return FALSE;
    }
    for(i = 0; i < cfg_desc->interface[1]. \
                  altsetting[num_altsetting-1].bNumEndpoints; i++)
    {
        if(endpoint[i].bEndpointAddress == BT_ISO_IN)
        {
            iso_pkt_size =  libusb_le16_to_cpu(endpoint[i].wMaxPacketSize);
            USBDBG("iso pkt size is %d", iso_pkt_size);
            iso_pkt_size_wh = iso_pkt_size * BT_MAX_ISO_FRAMES + \
                  sizeof(ISO_HDR);
            USBDBG("iso pkt size wh %d", iso_pkt_size_wh);
        }
   }
    return TRUE;
}


static int is_btusb_device (struct libusb_device *dev)
{
    struct bt_usb_device *id;
    struct libusb_device_descriptor desc;
    struct libusb_config_descriptor *cfg_desc;
    int    r, match, num_altsetting = 0;

    r = libusb_get_device_descriptor(dev, &desc);
    if (r < 0)
        return FALSE;

    match = 0;

    for (id = btusb_table; id->bDevClass; id++)
    {
        if (is_usb_match_idtable (id, &desc) == TRUE)
        {
                match = 1;
                break;
        }
    }

    if (!match)
    {
        return FALSE;
    }

    r = libusb_get_config_descriptor(dev, 0, &cfg_desc);
    if (r < 0)
    {
        USBERR("libusb_get_config_descriptor  %x:%x failed ....%d\n", \
               desc.idVendor, desc.idProduct, r);
        return FALSE;
    }

    r = check_bt_usb_endpoints(id, cfg_desc);
    libusb_free_config_descriptor(cfg_desc);

    return r;
}

/*******************************************************************************
**
** Function        libusb_open_bt_device
**
** Description     Scan the system USB devices. If match is found on
**                 btusb_table ensure that  it is a bluetooth device by
**                 checking Interface endpoint addresses.
**
** Returns         NULL: termination
**                 !NULL : pointer to the libusb_device_handle
**
*******************************************************************************/
static libusb_device_handle *libusb_open_bt_device()
{
    struct libusb_device **devs;
    struct libusb_device *found = NULL;
    struct libusb_device *dev;
    struct libusb_device_handle *handle = NULL;
    int    r, i;

    if (libusb_get_device_list(NULL, &devs) < 0)
    {
        return NULL;
    }
    for (i = 0; (dev = devs[i]) != NULL; i++)
    {
        if (is_btusb_device (dev) == TRUE)
            break;
    }
    if (dev)
    {
        r = libusb_open(dev, &handle);
        if (r < 0)
        {
            ALOGE("found USB BT device failed to open error = %d .....\n", r);
            return NULL;
        }
    }
    else
    {
        ALOGE("No matching USB BT device found .....\n");
        return NULL;
    }

    libusb_free_device_list(devs, 1);
    r = libusb_claim_interface(handle, 0);
    if (r < 0)
    {
        ALOGE("usb_claim_interface 0 error %d\n", r);
        return NULL;
    }

    intr_pkt_size = libusb_get_max_packet_size(dev, BT_INT_EP);
    USBDBG("Interrupt pkt size is %d", intr_pkt_size);
    intr_pkt_size_wh =  intr_pkt_size + sizeof(RX_HDR);

    r = libusb_claim_interface(handle, 1);
    if (r < 0)
    {
        ALOGE("usb_claim_interface 1 error %d\n", r);
    }

    return handle;
}

typedef enum {
  kCommandPacket = 1,
  kAclPacket = 2,
  kScoPacket = 3,
  kEventPacket = 4
} packet_type_t;

void handle_usb_events ()
{
    RX_HDR  *rx_buf;
    ISO_HDR *iso_buf;
    int  r, i, iso_xfer;
    struct libusb_transfer *transfer;
    struct timeval timeout = { 1, 0 };

    usb_xfer_status &= ~RX_DEAD;
    while (!(usb_xfer_status & RX_DEAD))
    {
        // This polling introduces two problems:
        //  1) /1s device wakeups when BT is on
        //  2) ~0.5s response to user's shutdown request
        libusb_handle_events_timeout(0, &timeout);
        transfer = NULL;
        iso_xfer = 0;
        if (usb_xfer_status & RX_FAILED)
        {
            if (data_rx_xfer->buffer == NULL)
            {
                transfer = data_rx_xfer;
                rx_buf = (RX_HDR *) bt_hc_cbacks->alloc(bulk_pkt_size_wh);
                if (rx_buf == NULL)
                {
                    USBERR("%s : Allocation failed", __FUNCTION__);
                    transfer = NULL;
                }
                else
                {
                    transfer->buffer = rx_buf->data;
                    transfer->length = bulk_pkt_size;
                }
            }
            else if (event_rx_xfer->buffer == NULL)
            {
                transfer = event_rx_xfer;
                rx_buf = (RX_HDR *) bt_hc_cbacks->alloc(intr_pkt_size_wh);
                if (rx_buf == NULL)
                {
                    USBERR("%s : Allocation failed", __FUNCTION__);
                    transfer = NULL;
                }
                else
                {
                    transfer->buffer = rx_buf->data;
                    transfer->length = intr_pkt_size;
                }
            }
            else if (iso_rx_xfer->buffer == NULL)
            {
                transfer = iso_rx_xfer;
                iso_buf = (ISO_HDR *) bt_hc_cbacks->alloc(iso_pkt_size_wh);
                if (iso_buf == NULL)
                {
                    USBERR("%s : Allocation failed", __FUNCTION__);
                    transfer = NULL;
                }
                else
                {
                    transfer->buffer = iso_buf->data;
                    transfer->length = BT_MAX_ISO_FRAMES * iso_pkt_size;
                    for(i = 0; i < transfer->num_iso_packets; i++)
                    {
                        transfer->iso_packet_desc[i].length = iso_pkt_size;
                    }
                    iso_xfer = 1;
                }
            }
            if (transfer != NULL)
            {
                usb_xfer_status &= ~(RX_FAILED);
                r = libusb_submit_transfer(transfer);
                if (r < 0)
                {
                    USBERR("libusb_submit_transfer : data_rx_xfer failed");
                    if (iso_xfer)
                    {
                        bt_hc_cbacks->free((TRANSAC) iso_buf);
                    }
                    else
                    {
                        bt_hc_cbacks->free((TRANSAC) rx_buf);
                    }
                    transfer->buffer = NULL;
                }
            }
         }
         else if (usb_xfer_status & XMIT_FAILED)
         {
             transfer = usb.failed_tx_xfer;
             USBDBG("Retransmitting xmit packet %d", \
                    *(transfer->buffer - 1));
             xmited_len = transfer->length;
             usb_xfer_status &= ~(XMIT_FAILED);
             if (libusb_submit_transfer(transfer) < 0)
            {
                USBERR("libusb_submit_transfer : %d : failed", \
                      *(transfer->buffer - 1));
            }
         }
      }
     usb_running = 0;
}


/*******************************************************************************
**
** Function        usb_read
**
** Description     Read data from the usb port
**
** Returns         Number of bytes actually read from the usb port and
**                 copied into p_data.  This may be less than len.
**
*******************************************************************************/
uint16_t  usb_read(uint16_t msg_id, uint8_t *p_buffer, uint16_t len)
{
    uint16_t total_len = 0;
    uint16_t copy_len = 0;
    uint8_t *p_data = NULL, iso_idx;
    int iso_frame_len = 0;
    int different_xfer = 0;
    struct iso_frames *frames;
    int pkt_rxing = 0;
    int rem_len = 0;
    ISO_HDR *p_iso_hdr;

#ifdef DUMPDATA
    int maxdata = 20;
    int lmax;
    char debugmessage[1024];
#endif
//    read_in_use = 0; // Reading

    if (!usb_running)
        return 0;
    while (total_len < len)
    {

        if (p_rx_hdr == NULL)
        {
            pthread_mutex_lock(&usb.mutex);

            if (usb.rxed_xfer < 0)
            {
                USBERR("Rx thread and usb_read out of sync %d", \
                       usb.rxed_xfer);
                usb.rxed_xfer = 0;
            }


            if (usb.rxed_xfer == 0 && usb.rx_status == RX_NEW_PKT)
            {
                usb.send_rx_event = TRUE;
                pthread_mutex_unlock(&usb.mutex);
                USBDBG("usb_read nothing to rx....");
//		read_in_use = 1;  // Reading last bytes ...
                return 0;

            }

            while (usb.rxed_xfer == 0)
            {
               pthread_cond_wait(&usb.cond, &usb.mutex);
            }
            usb.rxed_xfer--;
            pthread_mutex_unlock(&usb.mutex);

            if (usb.rx_status == RX_NEW_PKT)
            {
                p_rx_hdr = (RX_HDR *)utils_dequeue(&(usb.rx_eventq));
                if (p_rx_hdr == NULL)
                {
                    p_rx_hdr = (RX_HDR *)utils_dequeue(&(usb.rx_isoq));
                }
                if (p_rx_hdr == NULL)
                {
                    p_rx_hdr = (RX_HDR *)utils_dequeue(&(usb.rx_bulkq));
                }
                if (p_rx_hdr == NULL)
                {
                    USBERR("rxed_xfer is %d but no packet found", usb.rxed_xfer);
                    return 0;
                }
                switch (p_rx_hdr->event)
                {
                    case H4_TYPE_EVENT:
                        p_data = p_rx_hdr->data;
                        p_rx_hdr->offset = 0;
                        usb.rx_pkt_len = p_data[EV_LEN_FIELD] + \
                              HCI_EVT_PREAMBLE_SIZE;
                        usb.rx_status = RECEIVING_PKT;
                        *p_buffer = p_rx_hdr->event;
                        total_len += 1;
                        p_buffer++;
                        break;


                    case H4_TYPE_SCO_DATA:
                        p_iso_hdr = (ISO_HDR *)p_rx_hdr;
                        p_iso_hdr->offset = 0;
                        usb.rx_pkt_len = 0;
                        usb.rx_status = RECEIVING_PKT;
                        usb.iso_frame_ndx = 0;
                        pthread_mutex_lock(&usb.mutex);
                        usb.rxed_xfer++;
                        pthread_mutex_unlock(&usb.mutex);
                        break;


                    case H4_TYPE_ACL_DATA:
                        p_data = p_rx_hdr->data;
                        p_rx_hdr->offset = 0;
                        usb.rx_pkt_len = ((uint16_t)p_data[BLK_LEN_LO] | \
                              (uint16_t)p_data[BLK_LEN_HI] << 8) + \
                               HCI_ACL_PREAMBLE_SIZE;
                        usb.rx_status = RECEIVING_PKT;
                        *p_buffer = p_rx_hdr->event;
                        total_len += 1;
                        p_buffer++;
                        break;
                }
                USBDBG("A - Received packet from usb of len %d of type %x", \
                      p_rx_hdr->len, p_rx_hdr->event);



#ifdef DUMPDATA
		lmax = p_rx_hdr->len;
		sprintf(debugmessage, "REC1_DATA[%d] =", lmax);
		if (lmax > maxdata)
			lmax = maxdata;
		for (int i=0; i < lmax; i++) {
		  sprintf(debugmessage, "%s 0x%02x", debugmessage, p_data[i]);
		}
		ALOGE("%s", debugmessage);
#endif
            }
            else   // rx_status == RECIVING_PKT
            {
                switch (pkt_rxing)
                {
                    case H4_TYPE_EVENT:
                        p_rx_hdr = (RX_HDR *)utils_dequeue(&(usb.rx_eventq));
                        break;

                    case H4_TYPE_SCO_DATA:
                        p_rx_hdr = (RX_HDR *)utils_dequeue(&(usb.rx_isoq));
                        break;

                    case H4_TYPE_ACL_DATA:
                        p_rx_hdr = (RX_HDR *)utils_dequeue(&(usb.rx_bulkq));
                        break;
                 }
                 if (p_rx_hdr == NULL)
                 {
                     USBDBG("Rxed packet from different end_point.");
                     different_xfer++;
                 }
                 else
                 {
                     p_rx_hdr->offset = 0;
                     USBDBG("B - Received packet from usb of len %d of type %x",
                          p_rx_hdr->len, p_rx_hdr->event);
#ifdef DUMPDATA
		     lmax = p_rx_hdr->len;
		     sprintf(debugmessage, "REC2_DATA[%d] =", lmax);
		     if (lmax > maxdata)
			lmax = maxdata;
		     for (int i=0; i < lmax; i++) {
  			sprintf(debugmessage, "%s 0x%02x", debugmessage, p_data[i]);
		     }
		     ALOGE("%s", debugmessage);
#endif
                 }
             }
        }
        else //if (p_rx_hdr != NULL)
        {
            if (p_rx_hdr->event  == H4_TYPE_SCO_DATA)
            {
                p_iso_hdr = (ISO_HDR *)p_rx_hdr;
                frames = p_iso_hdr->frames;
                p_data = p_iso_hdr->data;
                frames += usb.iso_frame_ndx;
                if (usb.rx_pkt_len == 0) // Start of new micro frame
                {
                    if (frames->actual_length == 0)
                    {
	                 /* Previous frame has been processed */
                        usb.iso_frame_ndx++;
                        p_iso_hdr->offset = usb.iso_frame_ndx * iso_pkt_size;
                    }
                    *p_buffer = p_iso_hdr->event;
                    total_len += 1;
                    usb.rx_pkt_len = (uint16_t)p_data[p_iso_hdr->offset + \
                          SCO_LEN_FIELD] + HCI_SCO_PREAMBLE_SIZE;
                    p_buffer++;
                    usb.rx_status = RECEIVING_PKT;
                }
                else
                {
                    frames->actual_length -= len;
                }
                if (total_len == len)
                    break;
                pkt_rxing = p_iso_hdr->event;

                if((p_iso_hdr->len) <= (len - total_len))
                    copy_len = p_iso_hdr->len;
                else
                    copy_len = (len - total_len);

                p_iso_hdr->offset += copy_len;
                p_iso_hdr->len -= copy_len;
                rem_len = p_iso_hdr->len;
            }
            else
            {
                p_data = p_rx_hdr->data + p_rx_hdr->offset;
                pkt_rxing = p_rx_hdr->event;

                if((p_rx_hdr->len) <= (len - total_len))
                    copy_len = p_rx_hdr->len;
                else
                    copy_len = (len - total_len);

                p_rx_hdr->offset += copy_len;
                p_rx_hdr->len -= copy_len;
                rem_len = p_rx_hdr->len;
            }

            memcpy((p_buffer + total_len), p_data, copy_len);
            total_len += copy_len;

            if (rem_len == 0)
            {
               bt_hc_cbacks->free((TRANSAC) p_rx_hdr);
               p_rx_hdr = NULL;
            }
            usb.rx_pkt_len -= copy_len;
            if (usb.rx_pkt_len == 0)
            {
                usb.rx_status = RX_NEW_PKT;
		// Check if it is here the best place to set rx_enent to TRUE !!
                usb.send_rx_event = TRUE;
                pthread_mutex_unlock(&usb.mutex);

                break;
            }
            if (usb.rx_pkt_len < 0)
            {
                USBERR("pkt len expected %d rxed len of %d", len, total_len);
                usb.rx_status = RX_NEW_PKT;
                break;
            }
        }
    }
    if (different_xfer)
    {
        pthread_mutex_lock(&usb.mutex);
        usb.rxed_xfer += different_xfer;
        pthread_mutex_unlock(&usb.mutex);
    }

    return total_len;
}


/*******************************************************************************
**
** Function        usb_read_thread
**
** Description
**
** Returns         void *
**
*******************************************************************************/
static void *usb_read_thread(void *arg)
{
    RX_HDR  *rx_buf;
    ISO_HDR *iso_buf;
    int size, size_wh, r, i, iso_xfer;
    struct libusb_transfer *transfer;
    unsigned char *buf;

    LOG_ERROR("Entering usb_read_thread()");
    prctl(PR_SET_NAME, (unsigned long)"usb_read", 0, 0, 0);

//    while (thread_wait == 0) {
//        ALOGI("%s thread_wait", __func__);
//        utils_delay(50);
//    }

    rx_buf = (RX_HDR *) bt_hc_cbacks->alloc(bulk_pkt_size_wh);
    buf =  rx_buf->data;
    libusb_fill_bulk_transfer(data_rx_xfer, usb.handle, BT_BULK_IN, \
        buf, bulk_pkt_size, recv_xfer_cb, NULL, 0);
    r = libusb_submit_transfer(data_rx_xfer);
    if (r < 0)
    {
        USBERR("libusb_submit_transfer : data_rx_xfer : failed");
        goto out;
    }

    rx_buf = (RX_HDR *) bt_hc_cbacks->alloc(intr_pkt_size_wh);
    buf = rx_buf->data;
    libusb_fill_interrupt_transfer(event_rx_xfer, usb.handle, BT_INT_EP, \
          buf, intr_pkt_size, recv_xfer_cb, NULL, 0);
    r = libusb_submit_transfer(event_rx_xfer);
    if (r < 0)
    {
        USBERR("libusb_submit_transfer : event_rx_xfer : failed");
        goto out;
    }

    iso_buf =  (ISO_HDR *) bt_hc_cbacks->alloc(iso_pkt_size_wh);
    buf = iso_buf->data;
    libusb_fill_iso_transfer(iso_rx_xfer, usb.handle, BT_ISO_IN, buf, \
          iso_pkt_size * BT_MAX_ISO_FRAMES, BT_MAX_ISO_FRAMES, recv_xfer_cb, \
          NULL, 0);
    libusb_set_iso_packet_lengths (iso_rx_xfer, iso_pkt_size);

    usb_running = 1;
    handle_usb_events();
out:
    USBDBG("Leaving usb_read_thread()");
    if (data_rx_xfer != NULL)
    {
        rx_buf = CONTAINER_RX_HDR(data_rx_xfer->buffer);
        bt_hc_cbacks->free((TRANSAC) rx_buf);
        libusb_free_transfer(data_rx_xfer);
    }
    if (event_rx_xfer != NULL)
    {
        rx_buf = CONTAINER_RX_HDR(event_rx_xfer->buffer);
        bt_hc_cbacks->free((TRANSAC) rx_buf);
        libusb_free_transfer(event_rx_xfer);
    }
    if(iso_rx_xfer != NULL)
    {
        iso_buf = CONTAINER_ISO_HDR(iso_rx_xfer->buffer);
        bt_hc_cbacks->free((TRANSAC) iso_buf);
        libusb_free_transfer(iso_rx_xfer);
    }

    pthread_exit(NULL);

    return NULL;
}


static bool hal_open() {

    if (usb_running)
    {
        /* Userial is open; close it first */
        hal_close();
        utils_delay(50);
    }
    if (libusb_init(NULL) < 0)
    {
        ALOGE("libusb_init : failed");
        return FALSE;
    }

    usb.handle = libusb_open_bt_device();
    bulk_pkt_size_wh = BT_HCI_MAX_FRAME_SIZE + sizeof(RX_HDR);
    bulk_pkt_size = BT_HCI_MAX_FRAME_SIZE;
    if (usb.handle == NULL)
    {
        ALOGE("usb_open: HCI USB failed to open");
        goto out;
    }
    data_rx_xfer = libusb_alloc_transfer(0);
    if (!data_rx_xfer)
    {
        ALOGE("Failed alloc data_rx_xfer");
        goto out;
    }

    event_rx_xfer  = libusb_alloc_transfer(0);
    if (!event_rx_xfer)
    {
        ALOGE("Failed alloc event_rx_xfer");
        goto out;
    }

    iso_rx_xfer =  libusb_alloc_transfer(BT_MAX_ISO_FRAMES);
    if (!iso_rx_xfer)
    {
        ALOGE("Failed alloc iso_rx_xfer");
        goto out;
    }

    USBDBG("usb_read_thread is created ....");
    if (pthread_create(&(usb.read_thread), NULL, \
                       usb_read_thread, NULL) != 0 )
    {
        USBERR("pthread_create failed!");
        goto out;
    }

    stream_has_interpretation = false;
    stream_corruption_detected = false;
    stream_corruption_bytes_to_ignore = 0;

    hc_cb.worker_thread = thread_new("bt_hc_worker");
    if (!hc_cb.worker_thread) {
        ALOGE("%s unable to create worker thread.", __func__);
        return BT_HC_STATUS_FAIL;
    }

    return TRUE;
out :
    if (usb.handle != NULL)
    {
        if (data_rx_xfer != NULL)
            libusb_free_transfer(data_rx_xfer);
        if (event_rx_xfer != NULL)
            libusb_free_transfer(event_rx_xfer);
        if (iso_rx_xfer != NULL)
            libusb_free_transfer(iso_rx_xfer);
        libusb_release_interface(usb.handle, 1);
        libusb_release_interface(usb.handle, 0);
        libusb_close(usb.handle);
        libusb_exit(NULL);
    }

    return FALSE;

error:
  interface.close();
  return false;
}

static void hal_close() {
  LOG_INFO("%s", __func__);
}

size_t usb_read_data(serial_data_type_t type, uint8_t *buffer, size_t max_size, bool block) {
  if (type < DATA_TYPE_ACL || type > DATA_TYPE_EVENT) {
    LOG_ERROR("%s invalid data type: %d", __func__, type);
    return 0;
  } else if (!stream_has_interpretation) {
    LOG_ERROR("%s with no valid stream intepretation.", __func__);
    return 0;
  } else if (current_data_type != type) {
    LOG_ERROR("%s with different type than existing interpretation.", __func__);
    return 0;
  }

  return usb_read(type, buffer, max_size);
}
/*******************************************************************************
**
** Function        xmit_xfer_cb
**
** Description     Callback function after xmission
**
** Returns         None
**
**
*******************************************************************************/
static void xmit_xfer_cb(struct libusb_transfer *transfer)
{
    enum libusb_transfer_status status = transfer->status;
    static int xmit_acked;

    if(transfer->status != LIBUSB_TRANSFER_COMPLETED)
    {
        USBERR("xfer did not succeeded .....%d", transfer->status);
        usb_xfer_status |= XMIT_FAILED;
        usb.failed_tx_xfer = transfer;
        xmited_len = 0;
    } else
    {
        xmited_len = transfer->actual_length+1;
        libusb_free_transfer(transfer);
        usb_xfer_status  |= XMITTED;
        USBDBG("Xfer Succeded : count %d", ++xmit_acked);
    }
}

static void packet_finished(serial_data_type_t type) {
   uint8_t byte;
   uint8_t y = 0;
   uint8_t stop = 0;
   char debugmessage[1024*4];
  if (!stream_has_interpretation)
    LOG_ERROR("%s with no existing stream interpretation.", __func__);
  else if (current_data_type != type)
    LOG_ERROR("%s with different type than existing interpretation.", __func__);


//    read_in_use = 2; // no reading, so can write
    stream_has_interpretation = false;
}

/*******************************************************************************
**
** Function        usb_write
**
** Description     Write data to the usb port
**
** Returns         Number of bytes actually written to the usb port. This
**                 may be less than len.
**
*******************************************************************************/
uint16_t usb_write(uint16_t msg_id, uint8_t *p_data, uint16_t len)
{
    struct timeval tv = {60, 0};
    char buffer[512], pkt_type;;
    int i;
    CMD_HDR *cmd_hdr;
    static int xmit_count;
#ifdef DUMPDATA
    int lmax;
    int maxdata = 20;
    char debugmessage[1024*4];
#endif

    if (usb.handle == NULL) {
        return 0;
    }

    if(!(xmit_transfer = libusb_alloc_transfer(0)))
    {
        USBERR( "libusb_alloc_tranfer() failed");
        return 0;
    }

#ifdef DUMPDATA
    lmax = len;
    sprintf(debugmessage, "bt_hci WRITE_DATA[%d] =", lmax);
    if (lmax > maxdata)
	lmax = maxdata;
    for (int i=0; i < lmax; i++) {
      sprintf(debugmessage, "%s 0x%02x", debugmessage, p_data[i]);
    }
    ALOGE("%s HCI_LINK_KEY_REQUEST_EVT", debugmessage);
#endif

    pkt_type = *p_data;
    switch(pkt_type)
    {
        case H4_TYPE_COMMAND:
            /* Make use of BT_HDR space to populate setup */
            cmd_hdr = CONTAINER_CMD_HDR(p_data + 1);
            cmd_hdr->setup.bmRequestType = USB_TYPE_REQ;
            cmd_hdr->setup.wLength = len - 1;
            cmd_hdr->setup.wIndex = 0;
            cmd_hdr->setup.wValue = 0;
            cmd_hdr->setup.bRequest = 0;
            cmd_hdr->event = H4_TYPE_COMMAND;
            libusb_fill_control_transfer(xmit_transfer, usb.handle,
                (uint8_t *)&cmd_hdr->setup, xmit_xfer_cb, NULL, 0);
            break;

        case H4_TYPE_ACL_DATA:
            libusb_fill_bulk_transfer(xmit_transfer, usb.handle,
                BT_BULK_OUT, (p_data+1), (len-1), xmit_xfer_cb, NULL, 0);
            break;

        case H4_TYPE_SCO_DATA:
            libusb_fill_iso_transfer(xmit_transfer, usb.handle, \
                  BT_ISO_OUT, (p_data+1), (len-1), BT_MAX_ISO_FRAMES, \
                  xmit_xfer_cb, NULL, 0);
            break;

         default:
            USBERR("Unknown packet type to transmit %x", *p_data);
            return 0;
    }

    if (libusb_submit_transfer(xmit_transfer) < 0)
    {
        USBERR("libusb_submit_transfer : %d : failed", *p_data);
        return 0;
    }
    xmited_len = len;
    usb_xfer_status &= ~(XMITTED);

    while (!(usb_xfer_status & XMITTED)) {
        libusb_handle_events_timeout(0, &tv);
    }

    thread_wait = 1;
    return (xmited_len);
}

 static uint16_t transmit_data(serial_data_type_t type, uint8_t *data, uint16_t length) {
   if (type < DATA_TYPE_COMMAND || type > DATA_TYPE_SCO) {
     LOG_ERROR("%s invalid data type: %d", __func__, type);
     return 0;
   }
//   while (read_in_use < 2) {
//        utils_delay(5);
//   }   
 
   // Write the signal byte right before the data
   --data;
   uint8_t previous_byte = *data;
   *(data) = type;
   ++length;
 
   uint16_t transmitted_length = 0;
   while (length > 0) {
     ssize_t ret = usb_write(type, data + transmitted_length, length);
     switch (ret) {
       case -1:
         LOG_ERROR("In %s, error writing to the uart serial port: %s", __func__, strerror(errno));
         goto done;
       case 0:
         // If we wrote nothing, don't loop more because we
         // can't go to infinity or beyond
         goto done;
       default:
         transmitted_length += ret;
         length -= ret;
         break;
     }
   }
 
 done:;
   // Be nice and restore the old value of that byte
   *(data) = previous_byte;
 
   // Remove the signal byte from our transmitted length, if it was actually written
   if (transmitted_length > 0)
     --transmitted_length;
 
   return transmitted_length;
}

// Internal functions

// See what data is waiting, and notify the upper layer
static void event_usb_has_bytes(void) {

  if (stream_has_interpretation) {

    callbacks->data_ready(current_data_type);

  } else {
    uint8_t type_byte;

    if (type_byte < DATA_TYPE_ACL || type_byte > DATA_TYPE_EVENT) {
      LOG_ERROR("%s Unknown HCI message type. Dropping this byte 0x%x, min %x, max %x", __func__, type_byte, DATA_TYPE_ACL, DATA_TYPE_EVENT);
      return;
    }

    stream_has_interpretation = true;
    current_data_type = type_byte;
  }
}

static const hci_hal_t interface = {
  hal_init,           // init

  hal_open,           // open
  hal_close,          // close

  usb_read_data,      // read_data
  packet_finished,    // packet_finished
  transmit_data,      // transmit_data
};

const hci_hal_t *hci_hal_usb_get_interface() {
  bt_hc_cbacks = buffer_allocator_get_interface();
  return &interface;
}

