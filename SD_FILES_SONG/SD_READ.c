// sd_diskio.c

DRESULT SD_read(BYTE lun, BYTE *buff, DWORD sector, UINT count)
{
    if (count == 0 || buff == NULL)
    {
        return RES_PARERR;
    }

    if (SD_CheckStatusWithTimeout(SD_TIMEOUT) < 0)
    {
        return RES_NOTRDY;
    }

    if (BSP_SD_ReadBlocks(
            (uint32_t *)buff,
            (uint32_t)sector,
            (uint32_t)count,
            SD_TIMEOUT) != MSD_OK)
    {
        return RES_ERROR;
    }

    if (SD_CheckStatusWithTimeout(SD_TIMEOUT) < 0)
    {
        return RES_ERROR;
    }

    return RES_OK;
}