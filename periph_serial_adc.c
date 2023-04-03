/**
******************************************************************************
* @file     periph_serial_adc.c
* @author  zhy
* @brief   ads8688 config.
*          This is the common part of the sys periph initialization
*
   @verbatim
==============================================================================
##### How to use this driver #####
==============================================================================
[..]
The common HAL driver contains a set of generic and common APIs that can be
used by the PPP peripheral drivers and the user to start using the HAL.
[..]
The HAL contains two APIs' categories:
(+) Common HAL APIs
(+) Services HAL APIs

@endverbatim
******************************************************************************
* @attention None
*
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "periph_serial_adc.h"
#include <stdbool.h> 

/* Private typedef -----------------------------------------------------------*/
/** @defgroup AD acquisition Types
* @{
*/  

/**
* @brief  Struct def of ADS8688 SPI read
*/   
typedef struct 
{
  
  SpiClient_t *client;
  volatile uint32_t spiFlag;
  
  volatile uint8_t chx;
  
  volatile float raw[8];
  uint8_t range;
  volatile uint64_t lastUpdate;
  uint8_t error;
  uint8_t enabled;
  volatile bool atuo_acq_cmd; //in case spi time sequence chaos
  
} AD_Struct_t;  

/**
* @}
*/  

/* Private define ------------------------------------------------------------*/   
/* -----------------------------ADS8688 CMD Parameter-------------------------*/
#define AD_CMD_NO_OP               (0X0000)
#define AD_CMD_STANDBY             (0X8200)
#define AD_CMD_PW_DN               (0X8300)
#define AD_CMD_RST                 (0X8500)
#define AD_CMD_AUTO_RST            (0XA000)
#define AD_CMD_MAN_CHO             (0XC000)
#define AD_CMD_MAN_CH1             (0XC400)
#define AD_CMD_MAN_CH2             (0XC800)
#define AD_CMD_MAN_CH3             (0XCC00)
#define AD_CMD_MAN_CH4             (0XD000)
#define AD_CMD_MAN_CH5             (0XD400)
#define AD_CMD_MAN_CH6             (0XD800)
#define AD_CMD_MAN_CH7             (0XDC00)
#define AD_CMD_MAN_AUX             (0XE000)

/* -----------------------------ADS8688 REG ADDR-------------------------------*/
#define AD_SET_REGADDR(addr)    (addr << 1)|1
#define AD_GET_REGADDR(addr)    (addr << 1)|0

#define AD_REG_AUTO_SEQ_EN         (0X01)
#define AD_REG_CH_PW_DN            (0X02)
#define AD_REG_FEA_SEL             (0X03)
#define AD_REG_CH0_RANGE           (0X05)
#define AD_REG_CH1_RANGE           (0X06)
#define AD_REG_CH2_RANGE           (0X07)
#define AD_REG_CH3_RANGE           (0X08)
#define AD_REG_CH4_RANGE           (0X09)
#define AD_REG_CH5_RANGE           (0X0A)
#define AD_REG_CH6_RANGE           (0X0B)
#define AD_REG_CH7_RANGE           (0X0C)
#define AD_REG_CMD_BACK            (0X3F)

/* -----------------------------ADS8688 Periph--------------------------------*/
#define AD_V1_CS_PORT                   GPIOB
#define AD_V1_CS_PIN                    GPIO_PIN_12
#define AD_V1_RST_PORT                  GPIOD
#define AD_V1_RST_PIN                   GPIO_PIN_10   
#define AD_V1_RST_CLK_ENABLE()          __HAL_RCC_GPIOD_CLK_ENABLE()

#define AD_V2_CS_PORT                   GPIOA
#define AD_V2_CS_PIN                    GPIO_PIN_4
#define AD_V2_RST_PORT                  GPIOE
#define AD_V2_RST_PIN                   GPIO_PIN_9  
#define AD_V2_RST_CLK_ENABLE()          __HAL_RCC_GPIOE_CLK_ENABLE()

#define AD_C_CS_PORT                    GPIOE
#define AD_C_CS_PIN                     GPIO_PIN_11
#define AD_C_RST_PORT                   GPIOE
#define AD_C_RST_PIN                    GPIO_PIN_15 
#define AD_C_RST_CLK_ENABLE()          __HAL_RCC_GPIOE_CLK_ENABLE()

/* -----------------------------AD Current Extern  Resistance(¦¸)--------------*/
#define AD_EX_RESIS        250.0  

/* Private variables ---------------------------------------------------------*/
static AD_Struct_t  ad_v1_data = {0};
#pragma location = 0x24003800
static ALIGN_32BYTES (uint8_t ad_v1_rxbuf[4* AD_CHMAX]) = {0};

static AD_Struct_t  ad_v2_data = {0};
#pragma location = 0x24003840
static ALIGN_32BYTES (uint8_t ad_v2_rxbuf[4* AD_CHMAX]) = {0};

static AD_Struct_t  ad_c_data = {0};
#pragma location = 0x24003880
static ALIGN_32BYTES (uint8_t ad_c_rxbuf[4* AD_CHMAX]) = {0};

#pragma location = 0x240038C0
static ALIGN_32BYTES (uint8_t adx_txbuf[4]) = {0};
#pragma location = 0x24003910
static ALIGN_32BYTES (uint8_t adx_conftxbuf[4]) = {0};
#pragma location = 0x24003950
static ALIGN_32BYTES (uint8_t adx_confrxbuf[4]) = {0};

/* Private function prototypes -----------------------------------------------*/
static void periph_ad_rst_config(uint8_t ad_module);
static void periph_ad_set_atuo_rst_cmd(SpiClient_t *client, volatile uint32_t *flag);
static void periph_ad_set_cmd(SpiClient_t *client, uint16_t cmd, volatile uint32_t *flag);
static uint16_t periph_ad_get_cmd(SpiClient_t *client, volatile uint32_t *flag);
static bool periph_ad_verify_set_cmd(SpiClient_t *client, uint16_t cmd,volatile uint32_t *flag);

static void periph_ad_set_reg(SpiClient_t *client, uint8_t reg_addr, uint8_t value, \
                                                       volatile uint32_t *flag);
static uint8_t periph_ad_get_reg(SpiClient_t *client, uint8_t reg_addr ,\
                                                       volatile uint32_t *flag);
static bool periph_ad_verify_set_reg(SpiClient_t *client, uint8_t reg_addr, \
                                        uint8_t value ,volatile uint32_t *flag);

static bool periph_ad_config(SpiClient_t *client, uint8_t ad_range, volatile uint32_t *flag);

static void periph_ad_v1_callback(int unuse);
static void periph_ad_v2_callback(int unuse);
static void periph_ad_c_callback(int unuse);

static void periph_ad_decode(uint8_t *raw_data,AD_Struct_t *ad_data);
static float periph_ad_pars(uint16_t raw_data,uint8_t range);

/**
* @brief  ADS8688 init.  
* @param  ad_module  ADS8688 MODULE
           ad_range   AD_RAMGE
* @retval None    
*/
void Periph_Ad_Init(uint8_t ad_module,uint8_t ad_range)
{
  
  switch(ad_module){
    
    case AD_V1:   
    periph_ad_rst_config(ad_module);
    ad_v1_data.client = Bsp_Spi_Serial_Open(SPI2_ID, SPI_BAUDRATEPRESCALER_64, SPI_M2,\
                            AD_V1_CS_PORT, AD_V1_CS_PIN, &ad_v1_data.spiFlag, NULL);
    if(ad_v1_data.client == NULL || !periph_ad_config(ad_v1_data.client, ad_range,\
                                                           &ad_v1_data.spiFlag)){
        ad_v1_data.error = 1;
    }
    else{
          ad_v1_data.range = ad_range;
          ad_v1_data.enabled = 1;
          Bsp_Spi_Change_Callback(ad_v1_data.client,periph_ad_v1_callback); 
    } 
    break;
    
    case AD_V2:
    periph_ad_rst_config(ad_module);
    ad_v2_data.client = Bsp_Spi_Serial_Open(SPI1_ID, SPI_BAUDRATEPRESCALER_64, SPI_M2,\
                            AD_V2_CS_PORT, AD_V2_CS_PIN, &ad_v2_data.spiFlag, NULL);
    if(ad_v2_data.client == NULL || !periph_ad_config(ad_v2_data.client, ad_range,\
                                                           &ad_v2_data.spiFlag)){
        ad_v2_data.error = 1;
    }
    else{
          ad_v2_data.range = ad_range;
          ad_v2_data.enabled = 1;
          Bsp_Spi_Change_Callback(ad_v2_data.client,periph_ad_v2_callback); 
    } 
    break;  
    
    case AD_C:
    periph_ad_rst_config(ad_module);
    HAL_Delay(100);
    ad_c_data.client = Bsp_Spi_Serial_Open(SPI4_ID, SPI_BAUDRATEPRESCALER_64, SPI_M2,\
                            AD_C_CS_PORT, AD_C_CS_PIN, &ad_c_data.spiFlag, NULL);
    if(ad_c_data.client == NULL || !periph_ad_config(ad_c_data.client, ad_range,\
                                                           &ad_c_data.spiFlag)){
        ad_c_data.error = 1;
    }
    else{
          ad_c_data.range = ad_range;
          ad_c_data.enabled = 1;
          Bsp_Spi_Change_Callback(ad_c_data.client,periph_ad_c_callback); 
    } 
    break;
    
    default: 
    break;
  }
  
}

/**
* @brief  ADS8688 RST PIN .  
* @param  client     spi client
* @retval None    
*/
static void periph_ad_rst_config(uint8_t ad_module)
{
  
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  
  switch(ad_module){
    case AD_V1:
            AD_V1_RST_CLK_ENABLE();
            GPIO_InitStruct.Pin = AD_V1_RST_PIN;       
            HAL_GPIO_Init(AD_V1_RST_PORT, &GPIO_InitStruct);
            HAL_GPIO_WritePin(AD_V1_RST_PORT, AD_V1_RST_PIN, GPIO_PIN_SET);
    break;
    
    case AD_V2:
            AD_V2_RST_CLK_ENABLE();
            GPIO_InitStruct.Pin = AD_V2_RST_PIN;       
            HAL_GPIO_Init(AD_V2_RST_PORT, &GPIO_InitStruct);
            HAL_GPIO_WritePin(AD_V2_RST_PORT, AD_V2_RST_PIN, GPIO_PIN_SET); 
    break;  
    
    case AD_C:
            AD_C_RST_CLK_ENABLE();
            GPIO_InitStruct.Pin = AD_C_RST_PIN;       
            HAL_GPIO_Init(AD_C_RST_PORT, &GPIO_InitStruct);
            HAL_GPIO_WritePin(AD_C_RST_PORT, AD_C_RST_PIN, GPIO_PIN_SET); 
    break;
    
    default:
    break;
  }

}

/**
* @brief  ADS8688 config init.  
* @param  client      spi client
          ad_range    AD_RAMGE
* @retval None    
*/

static bool periph_ad_config(SpiClient_t *client, uint8_t ad_range, volatile uint32_t *flag)
{
  
  //reset program resgisters
  if(!periph_ad_verify_set_cmd(client, AD_CMD_RST, flag)){
    goto eror;
  }
  //Set voltage range
  if(!periph_ad_verify_set_reg(client, AD_REG_CH0_RANGE, ad_range, flag)){
    goto eror;
  }
  if(!periph_ad_verify_set_reg(client, AD_REG_CH1_RANGE, ad_range, flag)){
    goto eror;
  }
  if(!periph_ad_verify_set_reg(client, AD_REG_CH2_RANGE, ad_range, flag)){
    goto eror;
  }
  if(!periph_ad_verify_set_reg(client, AD_REG_CH3_RANGE, ad_range, flag)){
    goto eror;
  }
  if(!periph_ad_verify_set_reg(client, AD_REG_CH4_RANGE, ad_range, flag)){
    goto eror;
  }
  if(!periph_ad_verify_set_reg(client, AD_REG_CH5_RANGE, ad_range, flag)){
    goto eror;
  }  
  if(!periph_ad_verify_set_reg(client, AD_REG_CH6_RANGE, ad_range, flag)){
    goto eror;
  }  
  if(!periph_ad_verify_set_reg(client, AD_REG_CH7_RANGE, ad_range, flag)){
    goto eror;
  }
  periph_ad_set_atuo_rst_cmd(client,flag);
  return true;
eror:
  return false;
  
}

/**
* @brief  ADS8688  set command.  
* @param  client   spi client
* @retval None   
* @note except AUTO_RST cmd
*/
static void periph_ad_set_cmd(SpiClient_t *client, uint16_t cmd, volatile uint32_t *flag)
{ 
  
  uint16_t timeout = 0xfff;
  adx_conftxbuf[1] = (uint8_t)(cmd & 0x00ff);
  adx_conftxbuf[0] = (uint8_t)((cmd & 0xff00) >> 8);
  Bsp_Spi_Transaction(client,&adx_confrxbuf[0],&adx_conftxbuf[0],2);
  do{
    timeout--;
  }while((*flag == 0) && (timeout != 0)); 
  
}

/**
* @brief  ADS8688  get command.  
* @param  client   spi client
* @retval last cmd    
*/
static uint16_t periph_ad_get_cmd(SpiClient_t *client, volatile uint32_t *flag)
{
  
  uint16_t cmd = 0x00; 
  cmd = (((uint16_t)periph_ad_get_reg(client, AD_REG_CMD_BACK, flag)) << 8);  
  return cmd;
  
}

/**
* @brief  ADS8688  set cmd.  
* @param  client   spi client
value    set value
* @retval None  
* @note except AUTO_RST cmd  
*/
static bool periph_ad_verify_set_cmd(SpiClient_t *client, uint16_t cmd, volatile uint32_t *flag)
{
  
  uint8_t dec = 10;
  bool ret = true;
  do {
    periph_ad_set_cmd(client, cmd ,flag);
    dec--;
  } while (periph_ad_get_cmd(client, flag) != cmd && (dec != 0));
  
  if (dec == 0) {
    ret = false;
  }
  return ret;
  
}

/**
* @brief  ADS8688  set AUTO_RST command.  
* @param  client   spi client
* @retval None   
*/
static void periph_ad_set_atuo_rst_cmd(SpiClient_t *client, volatile uint32_t *flag)
{ 
  
  uint16_t timeout = 0xfff;
  adx_conftxbuf[1] = (uint8_t)(AD_CMD_AUTO_RST & 0x00ff);
  adx_conftxbuf[0] = (uint8_t)((AD_CMD_AUTO_RST & 0xff00) >> 8);
  Bsp_Spi_Transaction(client,&adx_confrxbuf[0],&adx_conftxbuf[0],4);
  do{
    timeout--;
  }while((*flag == 0) && (timeout != 0));  
  
}

/**
* @brief  ADS8688  Write program register.  
* @param  client   spi client
          reg_addr register address
          value    set value
* @retval None    
*/
static void periph_ad_set_reg(SpiClient_t *client, uint8_t reg_addr, uint8_t value, \
                                                         volatile uint32_t *flag)
{
  
  uint16_t timeout = 0xfff;
  adx_confrxbuf[0] = AD_SET_REGADDR(reg_addr);
  adx_confrxbuf[1] = value;
  Bsp_Spi_Transaction(client,&adx_confrxbuf[0],&adx_conftxbuf[0],3);
  do{
    timeout--;
  }while((*flag == 0) && (timeout != 0));
  
}

/**
* @brief  ADS8688   Read program register.  
* @param  client   spi client
          reg_addr register address
* @retval register  value  
*/
static uint8_t periph_ad_get_reg(SpiClient_t *client, uint8_t reg_addr, \
                                 volatile uint32_t *flag)
{

  uint16_t timeout = 0xfff;
  adx_conftxbuf[0] = AD_GET_REGADDR(reg_addr);
  Bsp_Spi_Transaction(client,&adx_confrxbuf[0],&adx_conftxbuf[0],3);
  do{
    timeout--;
  }while((*flag == 0) && (timeout != 0));
  return adx_confrxbuf[2];
}

/**
* @brief  ADS8688  Write program register.  
* @param  client   spi client
          reg_addr register address
          value    set value
* @retval None    
*/
static bool periph_ad_verify_set_reg(SpiClient_t *client, uint8_t reg_addr,\
                                     uint8_t value, volatile uint32_t *flag)
{
  uint8_t dec = 10;
  bool ret = true;
  do {
    periph_ad_set_reg(client, reg_addr, value, flag);
    dec--;
  } while (periph_ad_get_reg(client,reg_addr,flag) != value && (dec != 0));
  
  if (dec == 0) {
    ret = false;
  }
  return ret;
}

/**
* @brief  ADS8688  Periph_Ad_Transfer.  
* @param  ad_module   ADS8688 MODULE
* @retval None    
*/
void Periph_Ad_Transfer(uint8_t ad_module)
{
  
  switch(ad_module){
    case AD_V1:
      if(ad_v1_data.atuo_acq_cmd){
        periph_ad_set_atuo_rst_cmd(ad_v1_data.client,&ad_v1_data.spiFlag);
      }
      else{
         Bsp_Spi_Transaction(ad_v1_data.client, &ad_v1_rxbuf[ad_v1_data.chx * 4], \
                                                              &adx_txbuf[0], 4);
      }
    break;
    
    case AD_V2:
      if(ad_v2_data.atuo_acq_cmd){
        periph_ad_set_atuo_rst_cmd(ad_v2_data.client,&ad_v2_data.spiFlag);
      }
      else{
         Bsp_Spi_Transaction(ad_v2_data.client, &ad_v2_rxbuf[ad_v2_data.chx * 4], \
                                                              &adx_txbuf[0], 4); 
      }
    break;  
    
    case AD_C:
        if(ad_c_data.atuo_acq_cmd){
          periph_ad_set_atuo_rst_cmd(ad_c_data.client,&ad_c_data.spiFlag);
        }
        else{
          Bsp_Spi_Transaction(ad_c_data.client, &ad_c_rxbuf[ad_c_data.chx * 4], \
                                                              &adx_txbuf[0], 4); 
        }
    break;
    
    default:   
    break;
  }
  
}

/* -------------------------------------call back-----------------------------*/
/**
* @brief  ad v1 callback.  
*/
static void periph_ad_v1_callback(int unuse)
{
  if(ad_v1_data.atuo_acq_cmd == true){
    ad_v1_data.atuo_acq_cmd = false;
  }
  else{
    ad_v1_data.chx++;
    if(ad_v1_data.chx == AD_CHMAX){
      ad_v1_data.atuo_acq_cmd = true;  
    }
    ad_v1_data.chx %= AD_CHMAX;
    ad_v1_data.lastUpdate = HAL_GetTick();
  }
}

/**
* @brief  ad v2 callback.  
*/
static void periph_ad_v2_callback(int unuse)
{
  
  if(ad_v2_data.atuo_acq_cmd == true){
    ad_v2_data.atuo_acq_cmd = false;
  }
  else{
    ad_v2_data.chx++;
    if(ad_v2_data.chx == AD_CHMAX){
      ad_v2_data.atuo_acq_cmd = true;  
    }
    ad_v2_data.chx %= AD_CHMAX;
    ad_v2_data.lastUpdate = HAL_GetTick();
  }
}

/**
* @brief  ad c callback.  
*/
static void periph_ad_c_callback(int unuse)
{
  if(ad_c_data.atuo_acq_cmd == true){
    ad_c_data.atuo_acq_cmd = false;
  }
  else{
    ad_c_data.chx++;
    if(ad_c_data.chx == AD_CHMAX){
      ad_c_data.atuo_acq_cmd = true;  
    }
    ad_c_data.chx %= AD_CHMAX;
    ad_c_data.lastUpdate = HAL_GetTick();
  }
}

/* -------------------------------------data decode---------------------------*/
/**
* @brief  ad data Update.  
*/
void Periph_Ad_Update(uint8_t ad_module)
{
  
  switch(ad_module){
    case AD_V1:
       periph_ad_decode(&ad_v1_rxbuf[0], &ad_v1_data);
    break;
    
    case AD_V2:
        periph_ad_decode(&ad_v2_rxbuf[0], &ad_v2_data);
    break;  
    
    case AD_C:
        periph_ad_decode(&ad_c_rxbuf[0], &ad_c_data);
    break;
    
    default:
    break;
  }
  
}

/**
* @brief  ad data decode.  
*/
static void periph_ad_decode(uint8_t *raw_data, AD_Struct_t *ad_data)
{
  uint8_t index = 0;
  uint16_t ad_raw = 0;
  for(index = 0;index < AD_CHMAX;index++){
    ad_raw = ((uint32_t)raw_data[index * 4 + 2] << 8) | raw_data[index * 4 + 3];
    ad_data->raw[index] = periph_ad_pars(ad_raw, ad_data->range);
  }  
}

/**
* @brief  ad data decode.  
*/
static float periph_ad_pars(uint16_t raw_data, uint8_t range)
{
  float pars_value = 0.0;
  uint16_t  temp_data = 0;
  switch(range){
    case AD_RANGE_1:
    if(raw_data >= 0x8000){
      temp_data = raw_data - 0x8000;
      pars_value = temp_data /65536.0 * 20.48;
    }
    else{
      temp_data = 0x8000 - raw_data;
      pars_value = temp_data /65536.0 * 20.48 * (-1);
    }
    break;
    
    case AD_RANGE_2:
    if(raw_data >= 0x8000){
      temp_data = raw_data - 0x8000;
      pars_value = temp_data /65536.0 * 10.24;
    }
    else{
      temp_data = 0x8000 - raw_data;
      pars_value = temp_data /65536.0 * 10.24 * (-1);
    }
    break;  
    case AD_RANGE_3:
    if(raw_data >= 0x8000){
      temp_data = raw_data - 0x8000;
      pars_value = temp_data /65536.0 * 5.12;
    }
    else{
      temp_data = 0x8000 - raw_data;
      pars_value = temp_data /65536.0 * 5.12 * (-1);
    }
    break;
    case AD_RANGE_4:    
    pars_value = raw_data /65536.0 * 10.24;
    break;  
    case AD_RANGE_5:
    pars_value = raw_data /65536.0 * 5.12;
    break;
    
    default:
    break;
  }
  return pars_value;
}

/* -------------------------------------gat data-------------------------------*/
/**
* @brief  get error.  
*/
uint8_t Periph_Ad_Geterror(uint8_t ad_module)
{
  uint8_t error = 0;
  switch(ad_module){
    case AD_V1:
         error = ad_v1_data.error;
    break;
    
    case AD_V2:
         error = ad_v2_data.error;
    break;  
    
    case AD_C:
         error = ad_c_data.error;
    break;
    
    default:
    break;
  }
  return error;
}

/**
* @brief  get raw data. 
* @return  Voltage unit V
*          Current unit A
*/
float Periph_Ad_Getraw(uint8_t ad_module,uint8_t chx)
{
  float raw = 0;
  if(chx < AD_CHMAX){
    switch(ad_module){
      case AD_V1:
      raw = ad_v1_data.raw[chx];
      break;
      
      case AD_V2:
      raw = ad_v2_data.raw[chx];
      break;  
      
      case AD_C:
      raw = ad_c_data.raw[chx]/AD_EX_RESIS;
      break;
      default:
      break;
    }
  }
  return raw;
}
/********************************END OF FILE***********************************/
