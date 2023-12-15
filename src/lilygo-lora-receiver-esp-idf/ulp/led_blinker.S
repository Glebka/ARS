#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"

    // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/ulp_instruction_set.html
	/* Define variables, which go into .bss section (zero-initialized data) */
    .set    LED,    6
    .bss

    .global alerts_received
alerts_received:
    .long 0

//     .global led_state
// led_state:
//     .long 0

    /* Code goes into .text section */
    .text
    .global entry
entry:
	/* put code here */

    /* Read alerts counter */
    move r3, alerts_received
    ld r0, r3, 0
    and r0, r0, 0xFFFF

    /*if it is equal 0 then go to exit*/
    jump exit, eq

//    move r3, led_state
//    ld r0, r3, 0
//    add r0, r0, 1
//    st r0, r3, 0
//    
//    and r0, r0, 0x01
//    /* if the first bit of led_state is 0 then go to led_off */
//	jump led_off, eq

    WRITE_RTC_REG(RTC_GPIO_OUT_W1TS_REG,RTC_GPIO_OUT_DATA_W1TS_S+6,1,1)
    wait 60000
    WRITE_RTC_REG(RTC_GPIO_OUT_W1TC_REG,RTC_GPIO_OUT_DATA_W1TC_S+6,1,1)

	/* end the program */
    .global exit
exit:
    halt
