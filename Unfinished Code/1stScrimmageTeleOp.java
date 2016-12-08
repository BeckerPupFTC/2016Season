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

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import static java.lang.Math.sin;
import static java.lang.Math.cos;
import static java.lang.Math.abs;

/**
 * Linear Tele Op Mode
 * <p>
 * Enables control of the robot via the gamepad.
 * NOTE: This op mode will not work with the NXT Motor Controllers. Use an Nxt op mode
 * instead.                       
 */
 
public class LinearK9TeleOp extends LinearOpMode {
	//initialization
	DcMotor motorRight;
	DcMotor motorLeft;
	DcMotor intake;
	int totalDegrees; //This variable stores the orientation of the robot.
	int x; //This variable stores the x coordinate of the point between the back wheels of the robot.
	int y; //This variable stores the y coordinate of the point between the back wheels of the robot.
	
	@Override
	public void runOpMode() throws InterruptedException {
		//More initialization
		motorLeft = hardwareMap.dcMotor.get("left wheel");
		motorRight = hardwareMap.dcMotor.get("right wheel");
		intake = hardwareMap.dcMotor.get("intake");
	
		motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		
		totalDegrees = 0;
		x = 0;
		y = 0;
	
		waitForStart();
	
		}
	}	
	
	public void goDirection(int inches, int direction) { 
		/* For the variable direction, 0 makes the robot go straight, 1 right, -1 left,
		 * -2 makes a 180 turn to the left, 2 makes a 180 turn to the right. Larger numbers will
		 * make the robot spin in circles.
		 */
		double turning = direction * 2497.5; //2497.5 encoder units in 90 degrees
		motorLeft.setTargetPosition(motorLeft.getPosition() + turning);
		motorRight.setTargetPosition(motorRight.getPosition() - turning);
		totalDegrees = totalDegrees + direction;
		
		if (totalDegrees >= 4) {
		totalDegrees = totalDegrees - 4;
		}
		if (totalDegrees < 0) {
		totalDegrees = totalDegrees + 4;
		}// This ensures totalDegrees is always somewhere in 0 through 359.
		// Hopefully 0, 90, 180, or 270.
		
		motorLeft.setPower(1);
		motorRight.setPower(1);
		while(motorLeft.isBusy() || motorRight.isBusy()) {}

		double distance = inches * 229.183;
		switch (totalDegrees) {
			case 0:
				y = y + inches;
				break;
			case 1:
				x = x + inches;
				break;
			case 2:
				y = y - inches;
				break;
			case 3:
				x = x - inches;
				break;
			}
			
		
		motorLeft.setTargetPosition(motorLeft.getPosition() + distance);
		motorRight.setTargetPosition(motorRight.getPosition() + distance);
		while (motorLeft.isBusy() || motorRight.isBusy()) {}
		motorLeft.setPower(0);
		motorRight.setPower(0);
	}	
	public void goPlace(int x_coordinate, int y_coordinate) {
		x_distance = x_coordinate - x;
		y_distance = y_coordinate - y;
		
		switch (totalDegrees) {
			case 0:
				goDirection(0,2);
				break;
			case 1:
				goDirection(0,1);
				break;
			case 2:
				goDirection(0,2);
				break;
			case 3:
				goDirection(0,-1);
				break;
		}
		if (y_distance < 0) {
			goDirection(y_distance,2);
			goDirection(0,2);
		} else {
			goDirection(y_distance,0);
		}
		if (x_distance < 0) {
			goDirection(-x_distance,-1);
		} else {
			goDirection(x_distance,1);
		}
	}
}
//4995 encoder units/360 degrees
/* 2pi inches for 1440 encoder units. 1 inch is 229.183 units */

