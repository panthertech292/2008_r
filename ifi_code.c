/*******************************************************************************
* FILE NAME: ifi_code.c
*
* DESCRIPTION:
*  This file contains some useful functions that you can call in your program.
*
*******************************************************************************/

#include "ifi_frc.h"
#include "ifi_code.h"
#include "printf_lib.h"

extern unsigned char aBreakerWasTripped;
unsigned int knocker_downer = 0;
unsigned int arm = 0;
unsigned int switch_look = 0;
unsigned int suction_cup = 0;
unsigned int autocycle = 0;
int	false = 0;
int true = -1;
unsigned int counter_arm = 0;
/*******************************************************************************
* FUNCTION NAME: Default_Routine
* PURPOSE:       Performs the default mappings of inputs to outputs for the
*                Robot Controller.
* CALLED FROM:   this file, Process_Data_From_Master_uP routine
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void Default_Routine (void)
{
	static int	allow_vac=0;
	float Normal;
	static int up_disabled = 0;


	if ((Right_Joystick_y<YLower2) || (Right_Joystick_y>YUpper2)
			|| (Left_Joystick_y<YLower2)|| (Left_Joystick_y>YUpper2))
	{
		printf("Y movement\r");
		CIM_FrontLeft = Left_Joystick_y;
		CIM_BackLeft = Left_Joystick_y;
		CIM_FrontRight = 255-Right_Joystick_y;
		CIM_BackRight = Right_Joystick_y;
	}
	else if((Right_Joystick_x<LowerLimit) || (Right_Joystick_x>UpperLimit)	// note: may want to only use 1 stick
			|| (Left_Joystick_x<LowerLimit) || (Left_Joystick_x>UpperLimit))
	{
		printf("X movement\r");		// note: both fronts from left joystick, both rears from right
		CIM_FrontLeft = 255-Left_Joystick_x;
		CIM_BackLeft = Right_Joystick_x;
		CIM_FrontRight = 255-Left_Joystick_x;
		CIM_BackRight = 255-Right_Joystick_x;
	}
	else		//Left_Joystick and Right_Joystick in deadstick
	{
		CIM_FrontLeft=127;
		CIM_BackLeft=127;
		CIM_FrontRight=127;
		CIM_BackRight=127;
	}

    //printf("*******************************************\r");
    //printf("%d,%d,%d,%d\r",Left_Joystick_y,Left_Joystick_x,Right_Joystick_y,Right_Joystick_x);
    //printf("%d,%d,%d,%d\r",CIM_FrontLeft,CIM_BackLeft,CIM_FrontRight,CIM_BackRight);

	if( (Arm_Joystick_y >127) && (rc_dig_in01 == 0)) //switch closed and arm going down
	{
		CIM_Arm = 127;
		printf("Down Limit Switch activated\r");
		if((arm == 0)&&(Arm_Joystick_y > 200))
		{	counter_arm++;
			if((counter_arm >= 150)&&(counter_arm<160))
			{	printf("Override %d\r", counter_arm);
				CIM_Arm = 155;
			}
		}
		else
			counter_arm=0;
	}
	else if((Arm_Joystick_y < 127) && ((rc_dig_in02 == 0) || up_disabled )) //switch 2 closed and arm going up
	{
		up_disabled=true;
		CIM_Arm = 127;
		printf("Up Limit Switch activated\r");
	}
	else
	{
		if ((Arm_Joystick_y < UpperLimit) && (Arm_Joystick_y > LowerLimit))
		{
			CIM_Arm = 127;
		}
		else
		{
			if(rc_dig_in02 == 1) // up limit switch is not engaged
				up_disabled=false;
			CIM_Arm = Arm_Joystick_y;
		}
	}


	if(p4_sw_trig==1)                         //Pusher
	   {
		relay2_fwd=1;
		relay2_rev=0;
	   }
	else
	   {
		relay2_fwd=0;
		relay2_rev=1;
	   }
	if(p4_sw_top==1)                        //closer
	   {
		relay3_fwd=1;
		relay3_rev=0;
		arm = 1;
		}
	else
		{
		relay3_fwd=0;
		relay3_rev=1;
		arm=0;
		}
	if(p1_sw_aux2==1)                     //knockdown
	   {
		relay1_fwd=0;
		relay1_rev=1;
	   }
	else
	   {
		relay1_fwd=1;
		relay1_rev=0;
	   }

	 //*---------- Buttons to Relays----------------------------------------------

	  relay8_fwd = !rc_dig_in18;  /* Power pump only if pressure switch is off. */
	  relay8_rev=0;
}
 /*******************************************************************************
*
*	FUNCTION:		Update_OI_LEDs()
*
*	PURPOSE:		Updates the state of the various user
*					defined	LEDs on the operator interface.
*
*	CALLED FROM:
*
*	PARAMETERS:		None
*
*	RETURNS:		Nothing
*
*	COMMENTS:
*
*******************************************************************************/
void Update_OI_LEDs(void)
{
	if(user_display_mode == 0)
	{
		// update the "PWM 1" LED
    	if(p1_y >= 0 && p1_y <= 56)
    	{						// joystick is in full reverse position
      		Pwm1_green = 0;		// turn PWM1 green LED off
      		Pwm1_red = 1;		// turn PWM1 red LED on
		}
		else if(p1_y >= 125 && p1_y <= 129)
		{						// joystick is in neutral position
			Pwm1_green = 1;		// turn PWM1 green LED on
			Pwm1_red = 1;		// turn PWM1 red LED on
		}
		else if(p1_y >= 216 && p1_y <= 255)
		{						// joystick is in full forward position
			Pwm1_green = 1;		// turn PWM1 green LED on
			Pwm1_red = 0;		// turn PWM1 red LED off
		}
		else
		{						// in either forward or reverse position
			Pwm1_green = 0;		// turn PWM1 green LED off
			Pwm1_red = 0;		// turn PWM1 red LED off
		}

		// update the "PWM 2" LED
		if(p2_y >= 0 && p2_y <= 56)
		{						// joystick is in full reverse position
			Pwm2_green = 0;		// turn pwm2 green LED off
			Pwm2_red = 1;		// turn pwm2 red LED on
		}
		else if(p2_y >= 125 && p2_y <= 129)
		{						// joystick is in neutral position
			Pwm2_green = 1;		// turn PWM2 green LED on
			Pwm2_red = 1;		// turn PWM2 red LED on
		}
		else if(p2_y >= 216 && p2_y <= 255)
		{						// joystick is in full forward position
			Pwm2_green = 1;		// turn PWM2 green LED on
			Pwm2_red = 0;		// turn PWM2 red LED off
		}
		else
		{						// in either forward or reverse position
			Pwm2_green = 0;		// turn PWM2 green LED off
			Pwm2_red = 0;		// turn PWM2 red LED off
		}

		// update the "Relay 1" and "Relay 2" LEDs
		Relay1_green = relay1_fwd;	// LED is on when Relay 1 is FWD
		Relay1_red = relay1_rev;	// LED is on when Relay 1 is REV
		Relay2_green = relay2_fwd;	// LED is on when Relay 2 is FWD
		Relay2_red = relay2_rev;	// LED is on when Relay 2 is REV

		// update the "Switch 1", "Switch 2" and "Switch 3" LEDs
		Switch1_LED = !(int)rc_dig_in01;
		Switch2_LED = !(int)rc_dig_in02;
		Switch3_LED = !(int)rc_dig_in10;
	}
  	else  /* User Mode is On - displays data in OI 4-digit display*/
	{
		User_Mode_byte = backup_voltage * 10;
	}


} /* END Default_Routine(); */

