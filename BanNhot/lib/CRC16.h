/**
 * @brief
 * This method calculates CRC
 *
 * @return uint16_t calculated CRC value for the message
 * @ingroup buffer
 */
 uint16_t calcCRC16(uint8_t u8length,uint8_t *regs)
{
    unsigned int temp, temp2, flag;
	temp = 0xFFFF;
    for (unsigned char i = 0; i < u8length; i++)
    {
        temp = temp ^ regs[i];
        for (unsigned char j = 1; j <= 8; j++)
        {
            flag = temp & 0x0001;
            temp >>=1;
            if (flag)
                temp ^= 0xA001;
        }
    }
    // Reverse byte order.
    temp2 = temp >> 8;
    temp = (temp << 8) | temp2;
    temp &= 0xFFFF;
    // the returned value is already swapped
    // crcLo byte is first & crcHi byte is last
    return temp;
}