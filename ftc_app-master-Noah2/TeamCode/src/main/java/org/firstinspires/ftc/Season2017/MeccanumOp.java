/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.Season2017;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.lang.Math.*;

/**
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="UpgradedTeleOp", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class MeccanumOp extends LinearOpMode
{
    DcMotor wheelR;
    DcMotor wheelL;
    DcMotor intake;
    DcMotor launcher;
    Servo intakeServo;
    Servo beaconPresser;

    ElapsedTime runtime;


    @Override
    public void runOpMode() {
        wheelR = hardwareMap.dcMotor.get("wheelR");
        wheelL = hardwareMap.dcMotor.get("wheelL");
        intake = hardwareMap.dcMotor.get("launcher");
        launcher = hardwareMap.dcMotor.get("intake");
        intakeServo =  hardwareMap.servo.get("servo_1");
        beaconPresser = hardwareMap.servo.get("servo_2");

        wheelL.setDirection(DcMotorSimple.Direction.REVERSE);//This motor is pointing the wrong direction

        runtime = new ElapsedTime();

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        beaconPresser.setPosition(1);


        intake.setPower(0.5);
        //wheels
        while (opModeIsActive()) {

            check();
            //shooter
            if (gamepad1.a) {
                launcher.setPower(-0.36);
                runtime.reset();
                while(opModeIsActive() && runtime.seconds() < 3) {
                    check();
                }
                intakeServo.setPosition(1);
                runtime.reset();
                while (opModeIsActive() && runtime.seconds() < 2) {
                    check();
                }
                wheelL.setPower(0);
                wheelR.setPower(0);
                runtime.reset();
                while (opModeIsActive() && runtime.seconds() < 3) {
                    check();
                }
                intakeServo.setPosition(0);
                launcher.setPower(0);
            }
        }
    }
    public void check() {
        float throttle = gamepad1.left_stick_y;
        float direction = -gamepad1.left_stick_x;
        float right = throttle - direction;
        float left = throttle + direction;
        wheelR.setPower(right);
        wheelL.setPower(left);

        if (gamepad1.dpad_left) {
            beaconPresser.setPosition(0.2);
        }
        if (gamepad1.dpad_right) {
            beaconPresser.setPosition(1);
        }
        telemetry.addLine("...");
    }
}