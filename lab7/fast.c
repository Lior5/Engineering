/*



 * Main.c



 *



 * Main file containing the main state machine.



 */



/* Standard Includes */



#include <stdint.h>



#include <stdbool.h>



#include <stdio.h>







#include <ti/devices/msp432p4xx/driverlib/driverlib.h>







#include "Library/Clock.h"



#include "Library/Bump.h"



#include "Library/Motor.h"



#include "Library/Encoder.h"



#include "Library/Button.h"







#include "Library/HAL_I2C.h"



#include "Library/HAL_OPT3001.h"







#define TURN_TARGET_TICKS 119



#define DRIVE_TARGET_TICKS 200











void Initialize_System();







uint8_t bump_data;



uint8_t bump_data0;



uint8_t bump_data1;



uint8_t bump_data2;



uint8_t bump_data3;



uint8_t bump_data4;



uint8_t bump_data5;







int tick=0;







int mytime[20];



int i=0;



int mouse;







/* Variable for storing lux value returned from OPT3001 */



float lux;







typedef enum



{



    START = 0,



    WAIT,



    SETUP_DRIVE1,



    DRIVE1,



    STOP_DRIVE1,



    SETUP_TURN1,



    TURN1,



    STOP_TURN1,



    ALL_DONE



} my_state_t;







my_state_t state = START;







int main(void)








{



    bool left_done, right_done, back, right;



    int turnTicks, driveTicks;



    int left_encoder_zero_pos, right_encoder_zero_pos, count;







    Initialize_System();







    set_left_motor_pwm(0);



    set_right_motor_pwm(0);







    while (1)



    {



        // Read Bump data into a byte



        // Lower six bits correspond to the six bump sensors



        // put into individual variables so we can view it in GC



        bump_data = Bump_Read();



        //0 IS ON THE RIGHT SIDE



        bump_data0 = BUMP_SWITCH(bump_data,0);



        bump_data1 = BUMP_SWITCH(bump_data,1);



        bump_data2 = BUMP_SWITCH(bump_data,2);



        bump_data3 = BUMP_SWITCH(bump_data,3);



        bump_data4 = BUMP_SWITCH(bump_data,4);



        //5 IS ON THE LEFT SIDE



        bump_data5 = BUMP_SWITCH(bump_data,5);







        lux = OPT3001_getLux();







        // Emergency stop switch S2



        // Switch to state "STOP" if pressed



        if (button_S2_pressed()) 



        {



            state = ALL_DONE;



        }







        //-----------------------------------



        //        Main State Machine



        //-----------------------------------



        switch (state) 



        {







        case START:



            state = WAIT;



            break;







        case WAIT:



            if (button_S1_pressed()) 



            {


                count =0;
                state = SETUP_DRIVE1;



            }



            break;











        case SETUP_DRIVE1:

            left_encoder_zero_pos = get_left_motor_count();



            right_encoder_zero_pos = get_right_motor_count();





            driveTicks = 1400;

            set_left_motor_direction(true);



            set_right_motor_direction(true);



            set_left_motor_pwm(1);



            set_right_motor_pwm(.875);





            left_done = false;



            right_done = false;





            if(count==0)

            state = DRIVE1;

            if(count==1){

                right = true;

                left_encoder_zero_pos = get_left_motor_count();

                              right_encoder_zero_pos = get_right_motor_count();

                              back = false;



                              right =true;



                              turnTicks = 143;



                              driveTicks = 10;



                              state = SETUP_TURN1;


            }else if(count==2){

                driveTicks = 850;

                state = DRIVE1;

            }else if(count==3){

                right = true;


                driveTicks = 10;
                back = false;

                right = true;

                                left_encoder_zero_pos = get_left_motor_count();

                                              right_encoder_zero_pos = get_right_motor_count();

                                              back = true;



                                              right =true;



                                              turnTicks = 138;



                                              driveTicks = 20;



                                              state = SETUP_TURN1;


            }else if(count==4){

                driveTicks = 3000;

                state = DRIVE1;

            }

            break;







        case DRIVE1:

            if(abs(right_encoder_zero_pos-get_right_motor_count())>driveTicks){

                count++;

                state = SETUP_DRIVE1;

            }

            if(lux<120){



                set_left_motor_pwm(0);



                set_right_motor_pwm(0);



                state = WAIT;



                break;



            }



            set_left_motor_direction(true);



            set_right_motor_direction(true);



            set_left_motor_pwm(1);



            set_right_motor_pwm(.875);



            right = true;



            // 0 is on the right






            break;







        case SETUP_TURN1:



            if(!back){



                set_left_motor_direction(false);



                set_right_motor_direction(false);



                set_left_motor_pwm(.3);



                set_right_motor_pwm(.3);



                back = (get_left_motor_count() - left_encoder_zero_pos < -driveTicks);



                break;



            }



            left_encoder_zero_pos = get_left_motor_count();



            right_encoder_zero_pos = get_right_motor_count();



            if(right){



                set_left_motor_direction(true);



                set_right_motor_direction(false);



            }else{



                set_left_motor_direction(false);



                set_right_motor_direction(true);



            }







            left_done = false;



            right_done = false;







            state = TURN1;



            break;











        case TURN1:



            if (!left_done)



            {



                set_left_motor_pwm(.7);



                if(right)



                left_done = (get_left_motor_count() - left_encoder_zero_pos) > turnTicks;



                else left_done = (get_left_motor_count() - left_encoder_zero_pos) < -turnTicks;



            }



            else



            {



                set_left_motor_pwm(0);



            }







            if(!right_done)



            {



                set_right_motor_pwm(.7);



                if(right)



                right_done = (get_right_motor_count() - right_encoder_zero_pos) < -turnTicks;



                else right_done = (get_right_motor_count() - right_encoder_zero_pos) > turnTicks;



            }



            else



            {



                set_right_motor_pwm(0);



            }







            if (left_done && right_done) {



                set_left_motor_pwm(0);          // Stop all motors



                set_right_motor_pwm(0);

                count++;



                state = SETUP_DRIVE1;



            }



            break;







        case ALL_DONE:



            state = WAIT;



            break;







        } // end of case







        Clock_Delay1ms(10);



    }



}







void Initialize_System()



{



    /*



     * Initialize main clock



     *



     * SMCLK = 12Mhz



     */



    Clock_Init48MHz();







    /* Halting the Watchdog */



    MAP_WDT_A_holdTimer();







    /* Configuring GPIO LED1 as an output */



    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);







    /* Configure GPIO LED Red, LED Green, LED Blue */



    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0);



    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN1);



    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN2);







    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);







    Bump_Init();







    motor_init();







    encoder_init();







    button_init();







    MAP_SysCtl_enableSRAMBankRetention(SYSCTL_SRAM_BANK1);







    /*



     * Configuring SysTick to trigger at .001 sec (MCLK is 48Mhz)



     */



    MAP_SysTick_enableModule();



    MAP_SysTick_setPeriod(48000);



    MAP_SysTick_enableInterrupt();







    MAP_Interrupt_enableMaster();







    /* Initialize I2C communication */



    Init_I2C_GPIO();



    I2C_init();







    /* Initialize OPT3001 digital ambient light sensor */



    OPT3001_init();







    //__delay_cycles(100000);



}











/*



 * Handle the SysTick Interrupt.  Currently interrupting at 1/10 second.



 *



 * Increment the tick counter "tick"



 * Blink the red led



 */



void SysTick_Handler(void)



{



    tick++;



    // if ((tick%1000)==0) MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);        // Toggle RED LED each time through loop



}
