/**
 * @file ble_phy.c
 * @brief BLE PHY-layer bitstream processor implementation.
 *
 * See ble_phy.h for the public API and design notes.
 */

#include "ble_phy.h"
#include "ble_codec.h"
#include "bt_assigned_numbers.h"

#include <string.h>   /* memset, memcpy */
#include <stddef.h>   /* NULL */
#include <stdio.h>    /* printf, putchar */
#include <inttypes.h> /* PRIX32 */

/* ---------------------------------------------------------------------------
 * Public API implementation
 * ---------------------------------------------------------------------------*/

void ble_processor_init(ble_channel_processor_t *proc, uint8_t channel_index)
{
    if (!proc)
        return;

    memset(proc, 0, sizeof(*proc));
    ble_framer_init(&proc->framer, channel_index);
}

ble_status_t ble_push_bit(ble_channel_processor_t *proc, uint8_t bit)
{
    if (!proc)
        return BLE_ERROR;
    ble_framer_status_t status = ble_framer_push_bit(&proc->framer, bit);
    if (status == BLE_FRAMER_ERROR)
        return BLE_ERROR;
    if (status == BLE_FRAMER_SEARCHING)
        return BLE_SEARCHING;
    if (status == BLE_FRAMER_COLLECTING)
        return BLE_COLLECTING;

    uint8_t raw_pdu[BLE_PDU_MAX_BYTES + BLE_CRC_BYTES];
    unsigned int total_bytes = 0u;
    uint8_t preamble = 0u;
    if (ble_framer_get_frame(&proc->framer, raw_pdu, &total_bytes, &preamble) != 0)
        return BLE_ERROR;
    if (ble_decode_advertising_packet(raw_pdu, total_bytes, proc->framer.channel_index,
                                      preamble, &proc->last_packet) != 0)
        return BLE_ERROR;
    proc->packet_ready = 1;
    return BLE_VALID_PACKET;
}

int ble_get_packet(ble_channel_processor_t *proc, ble_packet_t *out)
{
    if (!proc || !out || !proc->packet_ready)
        return -1;

    memcpy(out, &proc->last_packet, sizeof(*out));
    proc->packet_ready = 0;
    return 0;
}

int ble_verify_crc(const ble_packet_t *pkt)
{
    if (!pkt)
        return 0;

    unsigned int payload_len = ble_payload_length_from_header(pkt->pdu);

    uint32_t computed = ble_crc_calc(pkt->pdu, 2u + payload_len, BLE_CRC_INIT_ADV);
    return (computed == (pkt->crc & 0xFFFFFFu)) ? 1 : 0;
}

/* ---------------------------------------------------------------------------
 * ble_print_packet — human-readable advertising packet decoder
 * ---------------------------------------------------------------------------*/

/* Advertising PDU type codes (Core Spec Vol 6, Part B, §2.3.1). */
#define PDU_ADV_IND 0x00u
#define PDU_ADV_DIRECT_IND 0x01u
#define PDU_ADV_NONCONN_IND 0x02u
#define PDU_SCAN_REQ 0x03u
#define PDU_SCAN_RSP 0x04u
#define PDU_CONNECT_IND 0x05u
#define PDU_ADV_SCAN_IND 0x06u

/* PDU header byte 0 field masks. */
#define PDU_HDR_TYPE_MASK 0x0Fu
#define PDU_HDR_TXADD (1u << 6)
#define PDU_HDR_RXADD (1u << 7)

/* AD type for Manufacturer Specific Data (Bluetooth Assigned Numbers §2.3). */
#define AD_TYPE_MANUF_SPEC 0xFFu

/* Random address subtype encoded in bits [47:46] (top 2 bits of MSB octet). */
#define RAND_ADDR_STATIC 0x03u /* 11b = Random Static                  */
#define RAND_ADDR_RPA 0x02u    /* 10b = Random Private Resolvable      */
#define RAND_ADDR_NRPA 0x00u   /* 00b = Random Private Non-Resolvable  */

/**
 * Print a 6-byte BLE device address (little-endian, addr[0] = LSB) in
 * standard colon-separated notation with its address type in parentheses.
 */
static void print_ble_addr(const uint8_t *addr, int is_random)
{
    printf("%02X:%02X:%02X:%02X:%02X:%02X",
           addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);

    if (is_random)
    {
        /* Subtype is encoded in the top 2 bits of the most significant octet. */
        uint8_t subtype = (addr[5] >> 6) & 0x03u;
        const char *desc;
        switch (subtype)
        {
        case RAND_ADDR_STATIC:
            desc = "Random Static";
            break;
        case RAND_ADDR_RPA:
            desc = "Random Private Resolvable";
            break;
        case RAND_ADDR_NRPA:
            desc = "Random Private Non-Resolvable";
            break;
        default:
            desc = "Random (Reserved)";
            break;
        }
        printf(" (%s)", desc);
    }
    else
    {
        printf(" (Public)");
    }
}

/**
 * Parse and print AdvData AD structures: raw hex, AD type labeling, and
 * Manufacturer Specific Data (AD type 0xFF) company lookup.
 */
static void print_adv_data(const uint8_t *data, unsigned int len)
{
    if (len == 0)
    {
        printf("AdvData  : (none)\n");
        return;
    }

    /* Raw hex dump. */
    printf("AdvData  :");
    for (unsigned int i = 0; i < len; i++)
        printf(" %02X", data[i]);
    printf("\n");

    /* Walk AD structures (length + type + value). */
    unsigned int i = 0;
    while (i < len)
    {
        uint8_t ad_len = data[i];
        if (ad_len == 0 || (i + 1u + ad_len) > len)
            break;

        uint8_t ad_type = data[i + 1u];
        unsigned int val_begin = i + 2u;
        unsigned int val_end = i + 1u + ad_len;

        printf("├─ Type  : 0x%02X (%s)\n", ad_type, bt_assigned_ad_type_name(ad_type));
        printf("├─ Data  :");
        if (val_end > val_begin)
        {
            for (unsigned int j = val_begin; j < val_end; j++)
                printf(" %02X", data[j]);
        }
        else
        {
            printf(" (none)");
        }
        printf("\n");

        if (ad_type == AD_TYPE_MANUF_SPEC && ad_len >= 3u)
        {
            /* Company ID is 2 bytes, little-endian. */
            uint16_t cid = (uint16_t)data[i + 2u] | ((uint16_t)data[i + 3u] << 8u);
            printf("Manuf    : %s (0x%04X)\n", bt_assigned_company_name(cid), cid);
        }

        i += 1u + ad_len;
    }
}

void ble_print_packet(const ble_packet_t *pkt)
{
    if (!pkt)
        return;

    uint8_t hdr0 = pkt->pdu[0];
    uint8_t pdu_type = hdr0 & PDU_HDR_TYPE_MASK;
    int tx_add = (hdr0 & PDU_HDR_TXADD) ? 1 : 0;
    int rx_add = (hdr0 & PDU_HDR_RXADD) ? 1 : 0;
    uint8_t pay_len = pkt->pdu[1];

    /* PDU type name and plain-English description. */
    const char *type_name;
    const char *type_desc;
    switch (pdu_type)
    {
    case PDU_ADV_IND:
        type_name = "ADV_IND";
        type_desc = "Connectable Undirected Advertising";
        break;
    case PDU_ADV_DIRECT_IND:
        type_name = "ADV_DIRECT_IND";
        type_desc = "Connectable Directed Advertising";
        break;
    case PDU_ADV_NONCONN_IND:
        type_name = "ADV_NONCONN_IND";
        type_desc = "Non-Connectable Undirected Advertising";
        break;
    case PDU_SCAN_REQ:
        type_name = "SCAN_REQ";
        type_desc = "Scan Request";
        break;
    case PDU_SCAN_RSP:
        type_name = "SCAN_RSP";
        type_desc = "Scan Response";
        break;
    case PDU_CONNECT_IND:
        type_name = "CONNECT_IND";
        type_desc = "Connect Request";
        break;
    case PDU_ADV_SCAN_IND:
        type_name = "ADV_SCAN_IND";
        type_desc = "Scannable Undirected Advertising";
        break;
    default:
        type_name = "RESERVED";
        type_desc = "Reserved/Unknown";
        break;
    }

    printf("[BLE Advertising Packet]\n");
    printf("PDU Type : %s (%s)\n", type_name, type_desc);

    switch (pdu_type)
    {
    case PDU_ADV_IND:
    case PDU_ADV_NONCONN_IND:
    case PDU_ADV_SCAN_IND:
    case PDU_SCAN_RSP:
    {
        /* Payload: AdvA (6 B) + AdvData (pay_len - 6 B) */
        if (pay_len < 6u)
        {
            printf("AdvA     : (payload too short)\n");
            break;
        }
        printf("AdvA     : ");
        print_ble_addr(&pkt->pdu[2], tx_add);
        printf("\n");
        print_adv_data(&pkt->pdu[8], pay_len > 6u ? pay_len - 6u : 0u);
        break;
    }

    case PDU_ADV_DIRECT_IND:
    {
        /* Payload: AdvA (6 B, TxAdd) + TargetA (6 B, RxAdd) */
        if (pay_len < 12u)
        {
            printf("AdvA     : (payload too short)\n");
            break;
        }
        printf("AdvA     : ");
        print_ble_addr(&pkt->pdu[2], tx_add);
        printf("\n");
        printf("TargetA  : ");
        print_ble_addr(&pkt->pdu[8], rx_add);
        printf("\n");
        break;
    }

    case PDU_SCAN_REQ:
    {
        /* Payload: ScanA (6 B, TxAdd) + AdvA (6 B, RxAdd) */
        if (pay_len < 12u)
        {
            printf("ScanA    : (payload too short)\n");
            break;
        }
        printf("ScanA    : ");
        print_ble_addr(&pkt->pdu[2], tx_add);
        printf("\n");
        printf("AdvA     : ");
        print_ble_addr(&pkt->pdu[8], rx_add);
        printf("\n");
        break;
    }

    case PDU_CONNECT_IND:
    {
        /* Payload: InitA (6 B, TxAdd) + AdvA (6 B, RxAdd) + LLData (22 B) */
        if (pay_len < 12u)
        {
            printf("InitA    : (payload too short)\n");
            break;
        }
        printf("InitA    : ");
        print_ble_addr(&pkt->pdu[2], tx_add);
        printf("\n");
        printf("AdvA     : ");
        print_ble_addr(&pkt->pdu[8], rx_add);
        printf("\n");
        break;
    }

    default:
    {
        /* Reserved/unknown — dump raw payload bytes. */
        printf("Payload  :");
        for (unsigned int i = 0; i < pay_len && (2u + i) < BLE_PDU_MAX_BYTES; i++)
            printf(" %02X", pkt->pdu[2u + i]);
        printf("\n");
        break;
    }
    }

    printf("CRC      : 0x%06" PRIX32 " [%s]\n",
           pkt->crc, ble_verify_crc(pkt) ? "PASS" : "FAIL");
}
