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
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.LegacyModule;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="UpgradedTeleOp", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
@Disabled
public class UpgradedTeleOp extends OpMode
{
    DcMotor wheelR;
    DcMotor wheelL;
    DcMotor intake;
    DcMotor launcher;
    Servo intakeServo;
    Servo beaconPresser;
    Servo sensorServo;
    LightSensor legoLightSensor;      // Primary LEGO Light sensor,
    OpticalDistanceSensor lightSensor;   // Alternative MR ODS sensor
    UltrasonicSensor rangeSensor;
    LegacyModule board;

    double servoPosition = 0.5;
    @Override
    public void init() {
        wheelR = hardwareMap.dcMotor.get("wheelR");
        wheelL = hardwareMap.dcMotor.get("wheelL");
        intake = hardwareMap.dcMotor.get("launcher");
        launcher = hardwareMap.dcMotor.get("intake");
        intakeServo =  hardwareMap.servo.get("servo_1");
        beaconPresser = hardwareMap.servo.get("servo_2");
        sensorServo = hardwareMap.servo.get("servo_3");
        wheelL.setDirection(DcMotorSimple.Direction.REVERSE);//This motor is pointing the wrong direction


        rangeSensor = hardwareMap.ultrasonicSensor.get("sensor_ultrasonic");
        board = hardwareMap.legacyModule.get("Legacy Module 1");
        board.enable9v(4, true);

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // get a reference to our Light Sensor object.
        legoLightSensor = hardwareMap.lightSensor.get("sensor_light");                // Primary LEGO Light Sensor
        lightSensor = hardwareMap.opticalDistanceSensor.get("sensor_ods");  // Alternative MR ODS sensor.

        // turn on LED of light sensor.
        lightSensor.enableLed(true);

    }
    @Override
    public void start() {
        beaconPresser.setPosition(1);
    }
    @Override
    public void loop () {
        //wheels
        float throttle = -gamepad1.left_stick_y;
        float direction = -gamepad1.left_stick_x;
        float right = throttle - direction;
        float left = throttle + direction;
        wheelR.setPower(right);
        wheelL.setPower(left);

        //intake
        if (gamepad1.x) {
            intake.setPower(0.5);
        }
        if (gamepad1.b) {
            intake.setPower(0);
        }

        //shooter
        //if (gamepad1.a) {
            //wheelL.setPower(0);
            //wheelR.setPower(0);
            //intake.setPower(0);
            //launcher.setPower(0.8);
            //try {
            //    java.lang.Thread.sleep(3000);
            //    intake_servo.setPosition(1);
            //    java.lang.Thread.sleep(3000);
            //    intake_servo.setPosition(0);
            //} catch (InterruptedException e) {
            //    intake_servo.setPosition(0);
            //}
            //launcher.setPower(0);
        //}
        if (gamepad1.a) {
            launcher.setPower(-0.3);
        }
        if (gamepad1.y) {
            launcher.setPower(0);
        }
        if (gamepad1.dpad_up) {
            intakeServo.setPosition(1);
        }
        if (gamepad1.dpad_down) {
            intakeServo.setPosition(0);
        }
        if (gamepad1.dpad_left) {
            beaconPresser.setPosition(0.2);
        }
        if (gamepad1.dpad_right) {
            beaconPresser.setPosition(1);
        }
        if (gamepad1.right_stick_y > 0.5) {
            servoPosition += 0.01;
        }
        if (gamepad1.right_stick_y < -0.5) {
            servoPosition -= 0.01;
        }
        sensorServo.setPosition(servoPosition);

        telemetry.addData("light sensor measurement:", legoLightSensor.getLightDetected());
        telemetry.addData("ods measurement:", lightSensor.getLightDetected());
        telemetry.addData("ultrasonic measurement: ", rangeSensor.getUltrasonicLevel());
        telemetry.addData("sensor servo position: ", sensorServo.getPosition());
        telemetry.update();
    }
}