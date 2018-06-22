#ifndef HAL_GPIO_DRIVER_H
#define HAL_GPIO_DRIVER_H

#include "stm32f4xx.h"                  // Added Target Hardware File
#include "stdint.h"



/*****************************************************************************/
/*																																					 */
/*												Macro definitions for GPIO Registers							 */
/*																																					 */
/*****************************************************************************/

/*
@brief: Macros for GPIO input mode select
*/


#define hal_gpio_input_mode								(unit32_t)0x00     // input mode select
#define hal_gpio_output_mode							(uint32_t)0x01     // output mode select
#define hal_gpio_alt_func_mode						(uint32_t)0x10		 // alternate function mode select
#define hal_gpio_analog_mode							(uint32_t)0x11     // Analog mode select


/*
@brief: Macros for GPIO Output Type select
*/

#define hal_gpio_ottype_push_pull         (uint32_t)0x00
#define hal_gpio_ottype_open_drain				(uint32_t)0x01


/*
@brief: Macros for GPIO Output Speed Register
*/

#define hal_gpio_otspeed_low							(uint32_t)0x00
#define hal_gpio_otspeed_Medium						(uint32_t)0x01
#define hal_gpio_otspeed_Fast							(uint32_t)0x10
#define hal_gpio_otspeed_High							(uint32_t)0x11


/*
@brief: Function Macros for Enabling and disabling RCC clock gating control

Note : These settings are specific to STM32F446RE
*/


#define HAL_GPIOA_RCC_ENABLE()  				((RCC->AHB1ENR)|= (1<<0))
#define HAL_GPIOB_RCC_ENABLE()  				((RCC->AHB1ENR)|= (1<<1))
#define HAL_GPIOC_RCC_ENABLE()  				((RCC->AHB1ENR)|= (1<<2))
#define HAL_GPIOD_RCC_ENABLE()  				((RCC->AHB1ENR)|= (1<<3))
#define HAL_GPIOE_RCC_ENABLE()  				((RCC->AHB1ENR)|= (1<<4))
#define HAL_GPIOF_RCC_ENABLE()  				((RCC->AHB1ENR)|= (1<<5))
#define HAL_GPIOG_RCC_ENABLE()  				((RCC->AHB1ENR)|= (1<<6))
#define HAL_GPIOH_RCC_ENABLE()  				((RCC->AHB1ENR)|= (1<<7))

	

/*

@brief: Macros for GPIO PORTS

Note: These settings are specific to STM32F446RE
*/


#define GPIO_PORT_A       GPIOA
#define GPIO_PORT_B       GPIOB
#define GPIO_PORT_C       GPIOC
#define GPIO_PORT_D       GPIOD
#define GPIO_PORT_E       GPIOE
#define GPIO_PORT_F       GPIOF
#define GPIO_PORT_G       GPIOG
#define GPIO_PORT_H       GPIOH


/*****************************************************************************/
/*																																					 */
/*												Macro definitions for Important DataStructures			*/
/*																																					 */
/*****************************************************************************/



typedef struct
{
	uint32_t pin;
	
	uint32_t mode;        // For seting gpio mode
	
	uint32_t ottype;      // For setting gpio setting the output type of gpio
	
	uint32_t otspeed;     // For setting gpio output speed setting. 
	
	uint32_t Alternate;
	
	uint32_t Pull;
	
}gpio_conf_t;





/***********************     Driver Exposed API's    ****************************/



/*
Description: Initializes the Desired GPIO Port
Parameters : GPIO Port and configuration structure for specific GPIO
Return Val : None

*/
void hal_gpio_init(GPIO_TypeDef *GPIOx,gpio_conf_t *conf);


/*
Description: Write a byte to a specificed pin of specified Port
Parameters : GPIO Port, Specific Pin_number and Value in byte
Return Val : None

*/

void hal_gpio_write_pin(GPIO_TypeDef *GPIOx,uint32_t pin, uint8_t val);

/*
Description: Read from Specific Pin of a specific Port
Parameters : GPIO Port, Specific Pin_number
Return Val : Byte Value returned.

*/

uint8_t hal_gpio_read_pin(GPIO_TypeDef *GPIOx,uint32_t pin);

/*
Description: Configuring Specified Pin of a specifed Port
Parameters : GPIO Port, Specific Pin_number and Alternate function
Return Val : None
*/

void hal_gpio_alt_func_pin(GPIO_TypeDef *GPIOx,uint32_t pin,uint16_t alt_func);















#endif