/**
 * @file    sdio_diskio.c
 * @brief   FatFS disk I/O driver for SDIO (4-bit, polling) on STM32F405
 *
 * Bus pins (WeAct STM32F405RGT6):
 *   PC8-11 = D0-D3, PC12 = CK, PD2 = CMD   (AF12, SDIO peripheral)
 *
 */

#include "sdio_diskio.h"
#include "debug_uart.h"
#include "stm32f4xx_hal.h"
#include "ff_gen_drv.h"

extern SD_HandleTypeDef hsd;

/* DMA completion flags — populated by callbacks, unused in polling mode.
 * Kept so link-time references from stm32f4xx_it.c resolve correctly. */
static volatile uint8_t s_rx_cplt   = 0;
static volatile uint8_t s_tx_cplt   = 0;
static volatile uint8_t s_dma_error = 0;

#define SDIO_TIMEOUT_MS       5000u
#define SDIO_BLOCK_SIZE       512u
/* After write, poll card state for up to this many ms before declaring error */
#define SDIO_WRITE_POLL_MS    2000u

static volatile DSTATUS s_stat = STA_NOINIT;

DSTATUS SD_disk_initialize(BYTE pdrv)
{
    (void)pdrv;

    /* Wait for card to reach transfer state (MX_SDIO_SD_Init already ran) */
    uint32_t t0 = HAL_GetTick();
    while (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER) {
        if ((HAL_GetTick() - t0) > 1000u) {
            DBG_Printf("[SDIO] Card not ready at disk_initialize\r\n");
            s_stat = STA_NOINIT;
            return s_stat;
        }
    }

    /* Switch to 4-bit bus */
/*     if (HAL_SD_ConfigWideBusOperation(&hsd, SDIO_BUS_WIDE_4B) != HAL_OK) {
        DBG_Printf("[SDIO] 4-bit bus config failed, continuing 1-bit\r\n");
    } else {
        DBG_Printf("[SDIO] 4-bit bus configured\r\n");
    } */
   DBG_Printf("[SDIO] Staying in 1-bit mode\r\n");

    /*
     * Speed upgrade: ClockDiv 118 → 2  (48 MHz / 4 = 12 MHz)
     * Stays within SD spec for Class 10 cards.
     * If your card is older/slower, change 2 to 10 (≈ 4.4 MHz).
     */
    SDIO->CLKCR = (SDIO->CLKCR & ~SDIO_CLKCR_CLKDIV_Msk) | (2u & SDIO_CLKCR_CLKDIV_Msk);
    DBG_Printf("[SDIO] Clock upgraded to %lu kHz\r\n",
               (unsigned long)(48000u / ((2u + 2u))));

    DBG_Printf("[SDIO] Running at initialization clock\r\n"); //abhi lagaya hai.

    s_stat = 0;
HAL_SD_CardInfoTypeDef info;

if (HAL_SD_GetCardInfo(&hsd, &info) == HAL_OK)
{
    DBG_Printf(
        "[SDIO] Init OK CardType=%lu BlockNbr=%lu BlockSize=%lu\r\n",
        (unsigned long)info.CardType,
        (unsigned long)info.LogBlockNbr,
        (unsigned long)info.LogBlockSize
    );
}
else
{
    DBG_Printf("[SDIO] HAL_SD_GetCardInfo failed\r\n");
}
    return s_stat;
}

DSTATUS SD_disk_status(BYTE pdrv)
{
    (void)pdrv;
    return s_stat;
}

DRESULT SD_disk_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count)
{
    (void)pdrv;
    if (s_stat & STA_NOINIT) return RES_NOTRDY;
    if (!count)               return RES_PARERR;

/*     DBG_Printf("[SDIO] Read sector=%lu count=%u\r\n",
               (unsigned long)sector, (unsigned)count);

    DBG_Printf(
    "[SDIO] Before read: state=%lu err=0x%08lX\r\n",
    (unsigned long)HAL_SD_GetCardState(&hsd),
    (unsigned long)HAL_SD_GetError(&hsd)); */
    // Ye pura section debug ke liye lagaya hai, abhi comment out kar diya hai. Agar read me problem aati hai to ye debug karne me help karega.


HAL_StatusTypeDef st =
    HAL_SD_ReadBlocks(&hsd,
                      buff,
                      sector,
                      count,
                      SDIO_TIMEOUT_MS);

if (st != HAL_OK)
{
    DBG_Printf(
        "[SDIO] Read FAILED status=%d state=%lu err=0x%08lX\r\n",
        st,
        (unsigned long)HAL_SD_GetCardState(&hsd),
        (unsigned long)HAL_SD_GetError(&hsd)
    );

    return RES_ERROR;
}  //Abhi lagaya hai.

    /* Confirm card returned to transfer state */
    uint32_t t0 = HAL_GetTick();
    while (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER) {
        if ((HAL_GetTick() - t0) > SDIO_TIMEOUT_MS) {
            DBG_Printf("[SDIO] Read post-poll timeout\r\n");
            return RES_ERROR;
        }
    }
    return RES_OK;
}

DRESULT SD_disk_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count)
{
    (void)pdrv;
    if (s_stat & STA_NOINIT) return RES_NOTRDY;
    if (!count)               return RES_PARERR;

//    DBG_Printf("[SDIO] Write sector=%lu count=%u\r\n", (unsigned long)sector, (unsigned)count); //Debug ke liye kaam aaya tha ye.

    if (HAL_SD_WriteBlocks(&hsd, (uint8_t *)buff, sector, count,
                           SDIO_TIMEOUT_MS) != HAL_OK)
    {
        DBG_Printf("[SDIO] Write FAILED err=0x%08lX\r\n",
                   (unsigned long)HAL_SD_GetError(&hsd));
        return RES_ERROR;
    }

    /*
     * Critical: poll until card returns to TRANSFER state.
     * Without this, a subsequent write starts while the card is still
     * busy programming, causing the next HAL_SD_WriteBlocks to fail
     * with HAL_SD_ERROR_TX_UNDERRUN even in pure polling mode.
     */
    uint32_t t0 = HAL_GetTick();
    while (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER) {
        if ((HAL_GetTick() - t0) > SDIO_WRITE_POLL_MS) {
            DBG_Printf("[SDIO] Write busy-poll timeout sector=%lu\r\n",
                       (unsigned long)sector);
            return RES_ERROR;
        }
    }
    return RES_OK;
}

DRESULT SD_disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
    (void)pdrv;
    if (s_stat & STA_NOINIT) return RES_NOTRDY;

    HAL_SD_CardInfoTypeDef info;

    switch (cmd) {
        case CTRL_SYNC:
            {
                uint32_t t0 = HAL_GetTick();
                while (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER) {
                    if ((HAL_GetTick() - t0) > SDIO_TIMEOUT_MS) return RES_ERROR;
                }
            }
            return RES_OK;

        case GET_SECTOR_COUNT:
            HAL_SD_GetCardInfo(&hsd, &info);
            *(DWORD *)buff = (DWORD)info.LogBlockNbr;
            return RES_OK;

        case GET_SECTOR_SIZE:
            *(WORD *)buff = (WORD)SDIO_BLOCK_SIZE;
            return RES_OK;

        case GET_BLOCK_SIZE:
            HAL_SD_GetCardInfo(&hsd, &info);
            *(DWORD *)buff = info.LogBlockSize / SDIO_BLOCK_SIZE;
            return RES_OK;

        case CTRL_TRIM:
            return RES_OK;

        default:
            return RES_PARERR;
    }
}

void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsd_p)
{
    (void)hsd_p;
    s_rx_cplt = 1;
}

void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd_p)
{
    (void)hsd_p;
    s_tx_cplt = 1;
}

void HAL_SD_ErrorCallback(SD_HandleTypeDef *hsd_p)
{
    (void)hsd_p;
    s_dma_error = 1;
    DBG_Printf("[SDIO] HAL_SD_ErrorCallback fired\r\n");
}

const Diskio_drvTypeDef SD_Driver = {
    SD_disk_initialize,
    SD_disk_status,
    SD_disk_read,
    SD_disk_write,
    SD_disk_ioctl,
};
