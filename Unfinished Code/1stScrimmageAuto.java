/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */
/* Important:
 * Stuff I need to check:
 * Direction of continuous servo;
 * Direction of intake;
 */


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.Range;

import static java.lang.Math.abs;

public class WreckChecker implements Runnable 
{
	int left_encoder_now = -2147483648;
	int right_encoder_now = -2147483648;
	int left_encoder_then;
	int right_encoder_then;
	volatile boolean inTrouble = false;
	
	WreckChecker(String name) 
	{
		thread = new Thread(this,name);
		thread.start();
	}
	
	public void run() 
	{
		while (weAreRunning) 
		{
			left_encoder_then = left_encoder_now;
			right_encoder_then = right_encoder_now;
			left_encoder_now = motorLeft.getPosition();
			right_encoder_now = motorRight.getPosition();
			if (left_encoder_then != -2147483648) 
			{	
				if ((motor_right.isBusy() && (abs(right_encoder_now - right_encoder_then) < 50)) || (motor_left.isBusy() && (abs(left_encoder_now - left_encoder_then) < 50))) 
				{
					try 
					{
						Thread.sleep(200);
					} 
					catch (Exception e) {}
					if ((motor_right.isBusy() && (abs(right_encoder_now - right_encoder_then) < 50)) || (motor_left.isBusy() && (abs(left_encoder_now - left_encoder_then) < 50))) 
					{
						inTrouble = true;
						try 
						{
							Thread.sleep(300);
						} 
						catch (Exception e) {}
						inTrouble = false;
					}
				}
			}
		}
	}
}

public class LinearK9TeleOp extends LinearOpMode {
	//initialization
	DcMotorController wheelController;
	DcMotor motorRight;
	DcMotor motorLeft;
	
	DcMotorController launcherController;
	DcMotor intake;
	DcMotor launcher;
	
	ServoController servoBox;
	Servo intakeServo;
	
	volatile boolean weAreRunning = true;
	
	WreckChecker wreckChecker;
	
	public double motorMax = 0.97; 
	/* Encoder software assumes max rpm
	 * of motor is 156; ours is 152, so I set it to this to
	 * not over power the motor.
	 */
	
	@Override
	public void runOpMode() 
	{
		//More initialization
		wheelController = hardwareMap.dcMotorController.get("wheel controller"); 
		motorLeft = hardwareMap.dcMotor.get("left wheel");
		motorRight = hardwareMap.dcMotor.get("right wheel");
		
		launcherController = hardwareMap.dcMotorController.get("launcher controller");
		intake = hardwareMap.dcMotor.get("intake");
		launcher = hardwareMap.dcMotor.get("launcher");
		
		servoBox = hardwareMap.servoController.get("servo box")
		intakeServo = hardwareMap.servo.get("servo 1");
	
		motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		
		waitForStart();
		
		wreckChecker = new WreckChecker("wreck checker");
		
		goDirection(33.5,0);
		intakeServo.setPosition(1);// If servo spins backwards, change to 0
		launcher.setPower(1);// If launcher spins backwards, change to -1
		try 
		{
			Thread.sleep(2000);
		}
		catch (Exception e) {}
		intakeServo.setPosition(0.5);
		launcher.setPower(0);
		goDirection(20,90);
		goDirection(22,-90);
		goDirection(20,-90);
	}
	
	public void goDirection(double inches, double degrees) 
	{ 
		boolean noError = true;
		/* For the variable direction, 0 makes the robot go straight, 90 right, -90 left,
		* -180 makes a 180 turn to the left, 180 makes a 180 turn to the right. Larger numbers will
		* make the robot spin in circles.
		*/
		int turnAmount = direction * 27.75; //27.75 encoder units in 1 degree
		
		leftTarget = motorLeft.getPosition() + turnAmount;
		rightTarget = motorRight.getPosition() - turnAmount;
		
		motorLeft.setTargetPosition(leftTarget);
		motorRight.setTargetPosition(rightTarget);
		
		motorLeft.setPower(motorMax);
		motorRight.setPower(motorMax);
		while(motorLeft.isBusy() || motorRight.isBusy()) 
		{
			if (inTrouble)
			{
				motorLeft.setPower(0);
				motorRight.setPower(0);
				
				leftNow = motorLeft.getPosition();
				rightNow = motorRight.getPosition();
				if (direction < 0)
				{
					goDirection(6,1);
				}
				if (direction > 0)
				{
					goDirection(6,-1);
				}
				
				try
				{
					Thread.sleep(500);
				}
				catch (Exception e) {}
				
				noError = false;
			}
		}
		double distance = inches * 229.183;
		
		if (noError)
		{
			motorLeft.setTargetPosition(motorLeft.getPosition() + distance);
			motorRight.setTargetPosition(motorRight.getPosition() + distance);
			motorLeft.setPower(motorMax);
			motorRight.setPower(motorMax);
			
			while (motorLeft.isBusy() || motorRight.isBusy()) 
			{
				if(inTrouble) 
				{
					motorLeft.setPower(0);
					motorRight.setPower(0);
					
					leftNow = motorLeft.getPosition();
					rightNow = motorRight.getPosition();
					
					goDirection(-6,0);
					
					try
					{
						Thread.sleep(1000);
					}	
					catch (Exception e) {}
					
					noError = false;
				}
			}
		}
		motorLeft.setPower(0);
		motorRight.setPower(0);
	}
}
//4995 encoder units/360 degrees
/* 2pi inches for 1440 encoder units. 1 inch is 229.183 units */

