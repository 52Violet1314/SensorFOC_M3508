/*************************************************************************************************
*   File Name            : bsp_spi_lan9252.c
*   File Descriptions    : low-level driver for ethercat slave(lan9252)
*   Kernel Architecture  : Cortex-M4
*   MCU Series           : STM32G474
*
*   (c) Copyright 2026-2036 iRobotTribe Technology Co.,Ltd.
*   All Rights Reserved.
*
*   iRobotTribe Confidential. This software is owned or controlled by iRobotTribe and may only be
*   used strictly in accordance with the applicable license terms. By expressly
*   accepting such terms or by downloading, installing, activating and/or otherwise
*   using the software, you are agreeing that you have read, and that you agree to
*   comply with and are bound by, such license terms. If you do not agree to be
*   bound by the applicable license terms, then you may not retain, install,
*   activate or otherwise use the software.
*
*   Revision History:
*
*   Version     Date          Author           Descriptions
*   ---------   ----------    ------------     ---------------
*   0.0.1       2026-3-17     iRobotTribe       First version;

*************************************************************************************************/

#include "main.h"
#include "spi.h"
#include "ecat_def.h"
#include "bsp_spi_lan9252.h"



#define Dummy_Byte                      0xFF
#define SPI_FAST_RW

/*******************************************************************************
* Function Name  : SPIWrite
* Description    : Wire byte data to lan9252
* Input          : - data: the data send to LAN9252
* Output         : none
* Return         : none: 
* Attention		 : None
*******************************************************************************/
void SPIWriteByte(uint8_t data)
{
	HAL_SPI_Transmit(&hspi3, &data, 1, 2000);
}
/*******************************************************************************
* Function Name  : SPIRead
* Description    : read byte data from lan9252
* Input          : none
* Output         : none
* Return         : data: read from lan9252
* Attention		 : None
*******************************************************************************/
uint8_t SPIReadByte(void)
{
    uint8_t data;   
    HAL_SPI_Receive(&hspi3, &data, 1, 2000);
    return (data);	
}

/*******************************************************************************
* Function Name  : SPIReadDWord
* Description    : read word data from lan9252
* Input          : Address：the addreoss to be read from lan9252
* Output         : none
* Return         : data: word data from lan9252
* Attention		 : None
*******************************************************************************/
uint32_t SPIReadDWord (uint16_t Address)
{
    UINT32_VAL dwResult;
    UINT16_VAL wAddr;
	
    wAddr.Val  = Address;
    //Assert CS line
    LAN9252_CS_LOW();

#ifdef SPI_FAST_RW 
    uint8_t tx_data[4],rx_data[4];	
    tx_data[0] = CMD_FAST_READ;
    tx_data[1] = wAddr.byte.HB;
    tx_data[2] = wAddr.byte.LB;
    tx_data[3] = CMD_FAST_READ_DUMMY;
    HAL_SPI_Transmit(&hspi3, tx_data, 4, 100);

    HAL_SPI_Receive(&hspi3, rx_data, 4, 10);
    dwResult.byte.LB = rx_data[0];
    dwResult.byte.HB = rx_data[1];
    dwResult.byte.UB = rx_data[2];
    dwResult.byte.MB = rx_data[3];
#else    
    //Write Command
     SPIWriteByte(CMD_FAST_READ);
    //Write Address
    SPIWriteByte(wAddr.byte.HB);
    SPIWriteByte(wAddr.byte.LB);
    //Dummy Byte
    SPIWriteByte(CMD_FAST_READ_DUMMY);
    
    //Read Bytes
    dwResult.byte.LB = SPIReadByte();
    dwResult.byte.HB = SPIReadByte();
    dwResult.byte.UB = SPIReadByte();
    dwResult.byte.MB = SPIReadByte();
#endif    
    //De-Assert CS line
    LAN9252_CS_HIGH();
   
    return dwResult.Val;
}

/*******************************************************************************
* Function Name  : SPIWriteDWord
* Description    : write Word lan9252 in 
* Input          : Address the address write to lan9252
										val：the data write to lan9252
* Output         : none
* Return         : none: 
* Attention		 : None
*******************************************************************************/
void SPIWriteDWord (uint16_t Address, uint32_t Val)
{
    UINT32_VAL dwData;
    UINT16_VAL wAddr;
    

    wAddr.Val  = Address;
    dwData.Val = Val;
    //Assert CS line
    LAN9252_CS_LOW();
    
#ifdef SPI_FAST_RW 
    uint8_t tx_data[7];
    tx_data[0] = CMD_SERIAL_WRITE;
    tx_data[1] = wAddr.byte.HB;
    tx_data[2] = wAddr.byte.LB;
    tx_data[3] = dwData.byte.LB;
    tx_data[4] = dwData.byte.HB;
    tx_data[5] = dwData.byte.UB;
    tx_data[6] = dwData.byte.MB;
    HAL_SPI_Transmit(&hspi3, tx_data, 7, 10);
#else
    //Write Command
    SPIWriteByte(CMD_SERIAL_WRITE);
    //Write Address
    SPIWriteByte(wAddr.byte.HB);
    SPIWriteByte(wAddr.byte.LB);
    //Write Bytes
    SPIWriteByte(dwData.byte.LB);
    SPIWriteByte(dwData.byte.HB);
    SPIWriteByte(dwData.byte.UB);
    SPIWriteByte(dwData.byte.MB);
#endif    

    //De-Assert CS line
    LAN9252_CS_HIGH();
}
/*******************************************************************************
* Function Name  : SPISendAddr
* Description    : write address to lan9252
* Input          : Address：the address write to lan9252
* Output         : none
* Return         : none: 
* Attention		 : None
*******************************************************************************/
void SPISendAddr (uint16_t Address)
{
    UINT16_VAL wAddr;    

    wAddr.Val  = Address;

#ifdef SPI_FAST_RW 
    uint8_t tx_data[2];    
    tx_data[0] = wAddr.byte.HB;
    tx_data[1] = wAddr.byte.LB;
    HAL_SPI_Transmit(&hspi3, tx_data, 2, 10);
#else
    //Write Address
    SPIWriteByte(wAddr.byte.HB);
    SPIWriteByte(wAddr.byte.LB);
#endif
}

/*******************************************************************************
* Function Name  : SPIReadBurstMode
* Description    : Read word from lan9252 in burst mode
* Input          : none
* Output         : none
* Return         : word data from lan9252
* Attention		 : None
*******************************************************************************/
UINT32 SPIReadBurstMode ()
{
    UINT32_VAL dwResult;

#ifdef SPI_FAST_RW
    uint8_t rx_data[4];    
    HAL_SPI_Receive(&hspi3, rx_data, 4, 10);
    dwResult.byte.LB = rx_data[0];
    dwResult.byte.HB = rx_data[1];
    dwResult.byte.UB = rx_data[2];
    dwResult.byte.MB = rx_data[3];
#else
    //Read Bytes
    dwResult.byte.LB = SPIReadByte();
    dwResult.byte.HB = SPIReadByte();
    dwResult.byte.UB = SPIReadByte();
    dwResult.byte.MB = SPIReadByte();    
#endif
    return dwResult.Val;
}

/*******************************************************************************
* Function Name  : SPIWriteBurstMode
* Description    : write data to lan9252 in burst mode
* Input          : val：the data write to lan9252
* Output         : none
* Return         : none: 
* Attention		 : None
*******************************************************************************/
void SPIWriteBurstMode (uint32_t Val)
{
    UINT32_VAL dwData;
    dwData.Val = Val;

#ifdef SPI_FAST_RW    
    uint8_t tx_data[4];
    tx_data[0] = dwData.byte.LB;
    tx_data[1] = dwData.byte.HB;
    tx_data[2] = dwData.byte.UB;
    tx_data[3] = dwData.byte.MB;
    HAL_SPI_Transmit(&hspi3, tx_data, 4, 10);
#else
    //Write Bytes
    SPIWriteByte(dwData.byte.LB);
    SPIWriteByte(dwData.byte.HB);
    SPIWriteByte(dwData.byte.UB);
    SPIWriteByte(dwData.byte.MB);
#endif
}


/*******************************************************************************
* Function Name  : SPIReadRegUsingCSR
* Description    : Read data from lan9252 use CSR
* Input          : ReadBuffer:data buf 
									 Address：the reg address write to lan9252
										Count:the number write to lan9252
* Output         : none
* Return         : none: 
* Attention		 : None
*******************************************************************************/
void SPIReadRegUsingCSR(uint8_t *ReadBuffer, uint16_t Address, uint8_t Count)
{
    UINT32_VAL param32_1 = {0};
    UINT8 i = 0;
    UINT16_VAL wAddr;
    wAddr.Val = Address;

    param32_1.v[0] = wAddr.byte.LB;
    param32_1.v[1] = wAddr.byte.HB;
    param32_1.v[2] = Count;
    param32_1.v[3] = ESC_READ_BYTE;

    SPIWriteDWord (ESC_CSR_CMD_REG, param32_1.Val);//send read operation

    do
    {
        param32_1.Val = SPIReadDWord (ESC_CSR_CMD_REG);
		
    }while(param32_1.v[3] & ESC_CSR_BUSY);//wait util send finished

    param32_1.Val = SPIReadDWord (ESC_CSR_DATA_REG);//read data

    
    for(i=0;i<Count;i++)
         ReadBuffer[i] = param32_1.v[i];//store data
   
    return;
}

/*******************************************************************************
* Function Name  : SPIWriteRegUsingCSR
* Description    : write data to lan9252 use CSR
* Input          : ReadBuffer:data buf 
									 Address：the reg address write to lan9252
										Count:the number write to lan9252
* Output         : none
* Return         : none: 
* Attention		 : None
*******************************************************************************/
void SPIWriteRegUsingCSR( uint8_t *WriteBuffer, uint16_t Address, uint8_t Count)
{
    UINT32_VAL param32_1 = {0};
    UINT8 i = 0;
    UINT16_VAL wAddr;

    for(i=0;i<Count;i++)
         param32_1.v[i] = WriteBuffer[i];

    SPIWriteDWord (ESC_CSR_DATA_REG, param32_1.Val);


    wAddr.Val = Address;

    param32_1.v[0] = wAddr.byte.LB;
    param32_1.v[1] = wAddr.byte.HB;
    param32_1.v[2] = Count;
    param32_1.v[3] = ESC_WRITE_BYTE;

    SPIWriteDWord (0x304, param32_1.Val);
    do
    {
        param32_1.Val = SPIReadDWord (0x304);

    }while(param32_1.v[3] & ESC_CSR_BUSY);

    return;
}


/*******************************************************************************
* Function Name  : SPIReadPDRamRegister
* Description    : read data from lan9252 pd ram
* Input          : ReadBuffer:data buf 
									 Address：the reg address write to lan9252
										Count:the number write to lan9252
* Output         : none
* Return         : none: 
* Attention		 : None
*******************************************************************************/
void SPIReadPDRamRegister(uint8_t *ReadBuffer, uint16_t Address, uint16_t Count)
{
    UINT32_VAL param32_1 = {0};
    UINT8 i = 0,nlength, nBytePosition;
    UINT8 nReadSpaceAvblCount;

//    /*Reset/Abort any previous commands.*/
    param32_1.Val = (unsigned long int)PRAM_RW_ABORT_MASK;                                                 

    SPIWriteDWord (PRAM_READ_CMD_REG, param32_1.Val);

    /*The host should not modify this field unless the PRAM Read Busy
    (PRAM_READ_BUSY) bit is a 0.*/
    do
    {
		param32_1.Val = SPIReadDWord (PRAM_READ_CMD_REG);
    }while((param32_1.v[3] & PRAM_RW_BUSY_8B));

    /*Write address and length in the EtherCAT Process RAM Read Address and
     * Length Register (ECAT_PRAM_RD_ADDR_LEN)*/
    param32_1.w[0] = Address;
    param32_1.w[1] = Count;
    SPIWriteDWord (PRAM_READ_ADDR_LEN_REG, param32_1.Val);

    /*Set PRAM Read Busy (PRAM_READ_BUSY) bit(-EtherCAT Process RAM Read Command Register)
     *  to start read operatrion*/
    param32_1.Val = PRAM_RW_BUSY_32B; /*TODO:replace with #defines*/
    SPIWriteDWord (PRAM_READ_CMD_REG, param32_1.Val);

    /*Read PRAM Read Data Available (PRAM_READ_AVAIL) bit is set*/
    do
    {			
		param32_1.Val = SPIReadDWord (PRAM_READ_CMD_REG);		
    }while(!(param32_1.v[0] & IS_PRAM_SPACE_AVBL_MASK));

    nReadSpaceAvblCount = param32_1.v[1] & PRAM_SPACE_AVBL_COUNT_MASK;

    /*Fifo registers are aliased address. In indexed it will read indexed data reg 0x04, but it will point to reg 0
     In other modes read 0x04 FIFO register since all registers are aliased*/

    /*get the UINT8 lenth for first read*/
    //Auto increment is supported in SPIO
    param32_1.Val = SPIReadDWord (PRAM_READ_FIFO_REG);
    nReadSpaceAvblCount--;
    nBytePosition = (Address & 0x03);
    nlength = (4-nBytePosition) > Count ? Count:(4-nBytePosition);
    memcpy(ReadBuffer+i ,&param32_1.v[nBytePosition],nlength);
    Count-=nlength;
    i+=nlength;

    //Lets do it in auto increment mode
    LAN9252_CS_LOW();
    //Write Command
    SPIWriteByte(CMD_FAST_READ);
    SPISendAddr(PRAM_READ_FIFO_REG);
    //Dummy Byte
    SPIWriteByte(CMD_FAST_READ_DUMMY);

    while(Count)
    {
        param32_1.Val = SPIReadBurstMode();

        nlength = Count > 4 ? 4: Count;
        memcpy((ReadBuffer+i) ,&param32_1,nlength);

        i+=nlength;
        Count-=nlength;
        nReadSpaceAvblCount --;
    }

    LAN9252_CS_HIGH();

    return;
}
/*******************************************************************************
* Function Name  : SPIWritePDRamRegister
* Description    : write data from lan9252 pd ram
* Input          : ReadBuffer:data buf 
									 Address：the reg address write to lan9252
										Count:the number write to lan9252
* Output         : none
* Return         : none: 
* Attention		 : None
*******************************************************************************/
void SPIWritePDRamRegister(uint8_t *WriteBuffer, uint16_t Address, uint16_t Count)
{
    UINT32_VAL param32_1 = {0};
    UINT8 i = 0,nlength, nBytePosition,nWrtSpcAvlCount;

//    /*Reset or Abort any previous commands.*/
    param32_1.Val = PRAM_RW_ABORT_MASK;                                                

    SPIWriteDWord (PRAM_WRITE_CMD_REG, param32_1.Val);

    /*Make sure there is no previous write is pending
    (PRAM Write Busy) bit is a 0 */
    do
    {			
        param32_1.Val = SPIReadDWord (PRAM_WRITE_CMD_REG);

    }while((param32_1.v[3] & PRAM_RW_BUSY_8B));

    /*Write Address and Length Register (ECAT_PRAM_WR_ADDR_LEN) with the
    starting UINT8 address and length)*/
    param32_1.w[0] = Address;
    param32_1.w[1] = Count;

    SPIWriteDWord (PRAM_WRITE_ADDR_LEN_REG, param32_1.Val);

    /*write to the EtherCAT Process RAM Write Command Register (ECAT_PRAM_WR_CMD) with the  PRAM Write Busy
    (PRAM_WRITE_BUSY) bit set*/

    param32_1.Val = PRAM_RW_BUSY_32B; /*TODO:replace with #defines*/
    SPIWriteDWord (PRAM_WRITE_CMD_REG, param32_1.Val);	

    /*Read PRAM write Data Available (PRAM_READ_AVAIL) bit is set*/
    do
    {	
       param32_1.Val = SPIReadDWord (PRAM_WRITE_CMD_REG);

    }while(!(param32_1.v[0] & IS_PRAM_SPACE_AVBL_MASK));

    /*Check write data available count*/
    nWrtSpcAvlCount = param32_1.v[1] & PRAM_SPACE_AVBL_COUNT_MASK;

    /*Write data to Write FIFO) */ 
    /*get the byte lenth for first read*/
    nBytePosition = (Address & 0x03);

    nlength = (4-nBytePosition) > Count ? Count:(4-nBytePosition);

    param32_1.Val = 0;
    memcpy(&param32_1.v[nBytePosition],WriteBuffer+i, nlength);

    SPIWriteDWord (PRAM_WRITE_FIFO_REG,param32_1.Val);

    nWrtSpcAvlCount--;
    Count-=nlength;
    i+=nlength;

    //Auto increment mode
    LAN9252_CS_LOW();

    //Write Command
    SPIWriteByte(CMD_SERIAL_WRITE);

    SPISendAddr(PRAM_WRITE_FIFO_REG);

    while(Count)
    {
        nlength = Count > 4 ? 4: Count;
        param32_1.Val = 0;
        memcpy(&param32_1, (WriteBuffer+i), nlength);

        SPIWriteBurstMode (param32_1.Val);
        i+=nlength;
        Count-=nlength;
        nWrtSpcAvlCount--;
    }

    LAN9252_CS_HIGH();
    return;
}


/*******************************************************************************
* Function Name  : SPIReadDRegister
* Description    : read reg from lan9252 pd ram
* Input          : ReadBuffer:data buf 
									 Address：the reg address write to lan9252
										Count:the number write to lan9252
* Output         : none
* Return         : none: 
* Attention		 : None
*******************************************************************************/
void SPIReadDRegister(uint8_t *ReadBuffer, uint16_t Address, uint16_t Count)
{
    if (Address >= 0x1000)
    {
         SPIReadPDRamRegister(ReadBuffer, Address,Count);
    }
    else
    {
         SPIReadRegUsingCSR(ReadBuffer, Address,Count);
    }
}

/*******************************************************************************
* Function Name  : SPIWriteRegister
* Description    : write reg from lan9252 pd ram
* Input          : ReadBuffer:data buf 
									 Address：the reg address write to lan9252
										Count:the number write to lan9252
* Output         : none
* Return         : none: 
* Attention		 : None
*******************************************************************************/
void SPIWriteRegister( uint8_t *WriteBuffer, uint16_t Address, uint16_t Count)
{
   
   if (Address >= 0x1000)
   {
		SPIWritePDRamRegister(WriteBuffer, Address,Count);
   }
   else
   {
		SPIWriteRegUsingCSR(WriteBuffer, Address,Count);
   }
    
}

/*
 * 描    述：LAN9252_ReadID
 * 功    能：CSR读操作读LAN9252的芯片ID
 * 入口参数：无
 * 出口参数：读到的芯片ID
 */
unsigned long LAN9252_ReadID(void)
{
	UINT8 Temp[10] = {0,0,0,0,0,0,0,0,0,0};	  
	SPIReadRegUsingCSR(Temp, 0x0e02, 2);	    
	return (Temp[0] | ((UINT32)Temp[1] << 8) | ((UINT32)Temp[2] << 16) | ((UINT32)Temp[3] << 24));	
	
}   
/*
 * 描    述：bsp_lan9252_test
 * 功    能：测试PDI接口
 * 入口参数：无
 * 出口参数：无
 */
uint8_t bsp_lan9252_test(void)
{
    unsigned long temp;
    uint8_t retries;

    temp = SPIReadDWord(0x64);
    for (retries = 0u; retries < 5u && temp != 0x87654321; retries++)
    {
        HAL_Delay(10);
        temp = SPIReadDWord(0x64);
    }
    if (temp != 0x87654321)
    {
        return 0u;
    }

    temp = LAN9252_ReadID();
    for (retries = 0u; retries < 5u && temp != 0x9252; retries++)
    {
        HAL_Delay(10);
        temp = LAN9252_ReadID();
    }
    return (temp == 0x9252) ? 1u : 0u;
}
/************************************ iRobotTribe (END OF FILE) ************************************/
