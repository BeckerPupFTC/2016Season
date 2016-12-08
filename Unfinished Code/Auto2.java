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


package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorController;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.hardware.DcMotorSimple;



@Autonomous(name="Auto",group="Autonomous")

public class Auto extends LinearOpMode {
   
	 //initialization
    DcMotorController wheelController;
    DcMotor wheelR;

    DcMotor wheelL;


    public double motorMax = 0.97;

	/* Encoder software assumes max rpm

	 * of motor is 156; ours is 152, so I set it to this to
	 
	 * not over power the motor.
	 	*/

    @Override
    public void runOpMode()
    {
        //More initialization
        wheelL = hardwareMap.dcMotor.get("wheelL");
        wheelR = hardwareMap.dcMotor.get("wheelR");
        wheelR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheelL.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        goDirection(33,0);
        //goDirection(20,90);
        //goDirection(22,-90);
        //goDirection(20,-90);
    }
    // direction = degrees times 4
    public void goDirection(int inches, int direction)
    {
		/* For the variable direction, 0 makes the robot go straight, 90 right, -90 left,
		* -180 makes a 180 turn to the left, 180 makes a 180 turn to the right. Larger numbers will
		* make the robot spin in circles.
		*/
        //int turnAmount = direction * 111; //27.75 encoder units in 1 degree, 111 in 4 degrees

        //int rightTarget = wheelR.getCurrentPosition() - turnAmount;

        //wheelR.setTargetPosition(rightTarget);

        //wheelR.setPower(motorMax);
        //if (direction > 0)
        //{
        //    wheelL.setPower(1);
        //}
        //else
        //{
        //    wheelL.setPower(-1);
        //}
        //while(wheelR.isBusy()) {}
        //wheelL.setPower(0);

        int distance = inches * 229;
        wheelR.setTargetPosition(wheelR.getCurrentPosition() + distance);
        if (inches > 0)
        {
            wheelL.setPower(1);
        }
        else
        {
            wheelL.setPower(-1);
        }
        wheelR.setPower(motorMax);

        while (wheelR.isBusy());
    }

}
//4995 encoder units/360 degrees
/* 2pi inches for 1440 encoder units. 1 inch is 229.183 units */