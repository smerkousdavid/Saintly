/*  
 * saintly.c - The most holy of wireless speaker
 * author: David Smerkous
 * date: 9/08/2018
 * license: MIT
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/dma-mapping.h>

/*
 * GPIOS
 */
#define WAN_LED 0x00 //The first led

/*
 * GPIO BLOCK
 * 
 * This gpio memory block is specific to the Atheros AR7242 processor
 * I used the following spec sheet for all of the GPIO regs https://datasheetspdf.com/pdf-file/912138/Atheros/AR7242/1
 * The APB base is the bus bridge initial block and the GPIO falls under the bus block
 */
#define BIT(X)   (1UL << (X))
#define APB_BASE  0x18000000 //The bus memory block
#define GPIO_BASE (APB_BASE + 0x00040000) //The gpio memory block
#define GPIO_OUT_ENABLE GPIO_BASE //The first address is the gpio enable registry
#define GPIO_OUT  (GPIO_BASE + 0x08) //The gpio out value registry
#define GPIO_CLEAR (GPIO_BASE + 0x10) //The gpio clear registry (clears all of the set bits, this is the same as calling &~)
#define GPIO_BLOCK_SIZE 4096

/*
 * GPIO MEMORY ADDR FUNCTIONS
 */
#define SET_GPIO_ENABLE(X) (GPIO_OUT_ENABLE & X)
#define SET_GPIO_DISABLE(X) (GPIO_OUT_CLEAR & ~X)
#define SET_GPIO_ON(X) (GPIO_OUT & X)
#define SET_GPIO_OFF(X) (GPIO_CLEAR & X)

/*
 * Self explanatory
 */
#define __PRE "saintly: "
#define log(I, X) printk(I __PRE X "\n")

static unsigned int pin = 0;

static int __init saintly_init(void) {
  (unsigned long *)(GPIO_BASE) = 0x01;
  log(KERN_INFO, "test");
  return 0;
}

static void __exit saintly_exit(void) {
  log(KERN_INFO, "Cleanup kernel module test\n");
}

module_init(saintly_init);
module_exit(saintly_exit);

MODULE_LICENSE("MIT");
MODULE_AUTHOR ("David Smerkous <smerkousdavid@gmail.com>");
MODULE_DESCRIPTION ("The most holy of wireless speakers");