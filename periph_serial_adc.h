/**
  ******************************************************************************
  * @file     periph_serial_adc.h
  * @author   zhy
  * @brief    ads8688 config.
  ******************************************************************************
  * @attention   None
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PERIPH_SERIAL_ADC_H
#define PERIPH_SERIAL_ADC_H

#ifdef __cplusplus
 extern "C" {
#endif
   
/* Includes ------------------------------------------------------------------*/
#include "bsp_spi_drv.h"  

/* Exported types ------------------------------------------------------------*/
/** @defgroup ADS8688 Exported Types
  * @{
  */  
   
   
   /**
  * @brief ADS8688 MODULE
  */
   
 typedef enum
 {
   
   AD_V1 = 0x00,      /*!<  voltage acquisition module 1 */ 
   AD_V2,             /*!<  voltage acquisition module 2 */ 
   AD_C,              /*!<  current acquisition module */ 
   
 }AD_MODULE;
 
/**
  * @brief  ADS8688 chx
*/
 typedef enum
 {
   
   AD_CH1,
   AD_CH2,
   AD_CH3,
   AD_CH4,
   
   AD_CH5,
   AD_CH6,
   AD_CH7,
   AD_CH8,
   
   AD_CHMAX,
 }AD_CHX; 
   
  
/**
  * @brief  ADS8688 voltage range
  * @note   Vref = 4.096V   

 INPUT RANGE(V) \ P FULLSCALE (V) \  N FULL SCALE(V) \  FULL SCALE (V) \ LSB(uv)
 +-2.5 * Vref   \      10.24      \       -10.24     \   20.48         \ 312.50
 +-1.25 * Vref  \      5.12       \       -5.12      \   10.24         \ 156.25
 +-0.625 * Vref \      2.56       \       -2.56      \   5.12          \ 78.125 
 0 ~ 2.5 * Vref \      10.24      \       0          \   10.24         \ 156.25
 0 ~ 1.25 * Vref \     5.12       \       0          \   5.12          \ 78.125

*/
 typedef enum
 {
   
   AD_RANGE_1,         /*!<  voltage range  -2.5 * Vref  ~ +2.5 * Vref  */     
   AD_RANGE_2,         /*!<  voltage range  -1.25 * Vref  ~ +1.25 * Vref  */  
   AD_RANGE_3,         /*!<  voltage range  -0.625 * Vref  ~ +0.625* Vref  */  
   AD_RANGE_4 = 0x05,  /*!<  voltage range    0 * Vref  ~ +2.5 * Vref     */  
   AD_RANGE_5,         /*!<  voltage range    0 * Vref   ~ +1.25 * Vref  */  

 }AD_RAMGE; 
   
/**
* @}
*/  

/* Peripheral Control functions  ************************************************/
void Periph_Ad_Init(uint8_t ad_module,uint8_t ad_range);
void Periph_Ad_Transfer(uint8_t ad_module);

void Periph_Ad_Update(uint8_t ad_module);
uint8_t Periph_Ad_Geterror(uint8_t ad_module);
float Periph_Ad_Getraw(uint8_t ad_module,uint8_t chx);

#ifdef __cplusplus
}
#endif



#endif 

/********************************END OF FILE***********************************/
