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


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="AutoLaunchRedDrive",group="AutoLaunch")

public class AutoLaunchRedDrive extends LinearOpMode {
   
	 //initialization
    DcMotor wheelR;
    DcMotor wheelL;
    DcMotor intake;
    DcMotor launcher;
    Servo intake_servo;
    //Servo beacon_presser;

    private ElapsedTime runtime = new ElapsedTime();

	/* Encoder software assumes max rpm
	 * of motor is 156; ours is 152, so I set it to this to
	 * not over power the motor.
	 */

    @Override
    public void runOpMode()
    {
        wheelR = hardwareMap.dcMotor.get("wheelR");
        wheelL = hardwareMap.dcMotor.get("wheelL");
        intake = hardwareMap.dcMotor.get("launcher");
        launcher = hardwareMap.dcMotor.get("intake");
        intake_servo =  hardwareMap.servo.get("servo_1");
        //beacon_presser = hardwareMap.servo.get("servo_2");
        wheelL.setDirection(DcMotorSimple.Direction.REVERSE);//This motor is pointing the wrong direction

        waitForStart();

        launcher.setPower(-0.33);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 6)) {
            telemetry.addLine("getting up to speed");
        }

        intake_servo.setPosition(1);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 6)) {
            telemetry.addLine("launching");
        }

        intake.setPower(0.5);
        launcher.setPower(-0.32);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 6)) {
            telemetry.addLine("launching");
        }
        launcher.setPower(0);
        intake_servo.setPosition(0);
        intake.setPower(0);
    }

}
//4995 encoder units/360 degrees
/* 2pi inches for 1440 encoder units. 1 inch is 229.183 units */