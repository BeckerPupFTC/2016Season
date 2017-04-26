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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LegacyModule;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="RedBeacongb", group="Beacon")
//@Disabled
public class RedBeacon3 extends LinearOpMode {

    /* Declare OpMode members. */
    DcMotor rightMotor;
    DcMotor leftMotor;
    DcMotor intake;
    DcMotor launcher;
    Servo intakeServo;
    Servo beaconPresser;
    Servo sensorServo;
    LightSensor legoLightSensor;      // Primary LEGO Light sensor,
    OpticalDistanceSensor   lightSensor;   // Alternative MR ODS sensor
    UltrasonicSensor rangeSensor;
    LegacyModule board;

    ElapsedTime r = new ElapsedTime();
    ElapsedTime ru = new ElapsedTime();

    static final double     WHITE_THRESHOLD = 0.15;  // spans between 0.1 - 0.5 from dark to light

    @Override
    public void runOpMode() {

        rightMotor = hardwareMap.dcMotor.get("wheelR");
        leftMotor = hardwareMap.dcMotor.get("wheelL");
        intake = hardwareMap.dcMotor.get("launcher");
        launcher = hardwareMap.dcMotor.get("intake");
        intakeServo = hardwareMap.servo.get("servo_1");
        beaconPresser = hardwareMap.servo.get("servo_2");
        sensorServo = hardwareMap.servo.get("servo_3");
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);//This motor is pointing the wrong direction

        rangeSensor = hardwareMap.ultrasonicSensor.get("sensor_ultrasonic");
        board = hardwareMap.legacyModule.get("Legacy Module 1");
        board.enable9v(4, true);

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // get a reference to our Light Sensor object.
        legoLightSensor = hardwareMap.lightSensor.get("sensor_light");                // Primary LEGO Light Sensor
        lightSensor = hardwareMap.opticalDistanceSensor.get("sensor_ods");  // Alternative MR ODS sensor.

        // turn on LED of light sensor.
        lightSensor.enableLed(true);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        // Abort this loop is started or stopped.
        while (!(isStarted() || isStopRequested())) {

            // Display the light level while we are waiting to start
            telemetry.addData("Light Level", lightSensor.getLightDetected());
            telemetry.addData("distance: ", rangeSensor.getUltrasonicLevel());
            telemetry.update();
            idle();
        }
        sensorServo.setPosition(0.11);
        beaconPresser.setPosition(1);
        intake.setPower(0.5);

        // Start the robot moving forward, and then begin looking for a white line.
        leftMotor.setPower(0.88);
        rightMotor.setPower(0.8);
        while (opModeIsActive() && (rangeSensor.getUltrasonicLevel() > 13 || rangeSensor.getUltrasonicLevel() < 2)) {
            telemetry.addData("Light Level", lightSensor.getLightDetected());
            telemetry.addData("distance: ", rangeSensor.getUltrasonicLevel());
            telemetry.update();
        }
        leftMotor.setPower(0);
        sensorServo.setPosition(0.36);

        delay(r, 1);

        while (opModeIsActive() && (rangeSensor.getUltrasonicLevel() < 6 || rangeSensor.getUltrasonicLevel() == 0)) {
            telemetry.addData("Light Level: ", lightSensor.getLightDetected());
            telemetry.addData("distance: ", rangeSensor.getUltrasonicLevel());
            telemetry.update();
        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);
        delay(r, 0.4);
        leftMotor.setPower(-0.8);
        rightMotor.setPower(-0.8);
        delay(r, 0.5);
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        delay(r, 0.3);
        leftMotor.setPower(0.8);
        rightMotor.setPower(0.8);


        // run until the white line is seen OR the driver presses STOP;
        while (opModeIsActive() && (lightSensor.getLightDetected() < WHITE_THRESHOLD)) {

            telemetry.addData("distance: ", rangeSensor.getUltrasonicLevel());
            telemetry.addData("Light Level", lightSensor.getLightDetected());
            telemetry.update();
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        beaconPresser.setPosition(0.5);
        delay(r, 0.4);


        leftMotor.setPower(-1);
        rightMotor.setPower(-1);
        delay(r, 0.3);
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        delay(r, 0.2);

        double lightLevel = legoLightSensor.getLightDetected();

        leftMotor.setPower(1);
        rightMotor.setPower(1);
        delay(r, 0.4);
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        delay(r, 0.2);
        if (legoLightSensor.getLightDetected() < lightLevel) {
            beaconPresser.setPosition(0.6);
            leftMotor.setPower(-1);
            rightMotor.setPower(-1);
            delay(r, 0.4);
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }

        ru.reset();
        while (opModeIsActive() && ru.seconds() < 2) {
            beaconPresser.setPosition(0.6);
            delay(r, 0.4);
            beaconPresser.setPosition(0.2);
            delay(r, 0.4);
        }
        beaconPresser.setPosition(1);
        delay(r, 0.4);

        leftMotor.setPower(0.76);
        rightMotor.setPower(0.8);

        delay(r, 0.4);

        while (opModeIsActive() && (lightSensor.getLightDetected() < WHITE_THRESHOLD)) {

            telemetry.addData("distance: ", rangeSensor.getUltrasonicLevel());
            telemetry.addData("Light Level", lightSensor.getLightDetected());
            telemetry.update();
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        beaconPresser.setPosition(0.5);
        delay(r, 0.4);


        leftMotor.setPower(-1);
        rightMotor.setPower(-1);
        delay(r, 0.3);
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        delay(r, 0.2);

        lightLevel = legoLightSensor.getLightDetected();

        leftMotor.setPower(1);
        rightMotor.setPower(1);
        delay(r, 0.4);
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        delay(r, 0.2);
        if (legoLightSensor.getLightDetected() < lightLevel) {
            beaconPresser.setPosition(0.6);
            leftMotor.setPower(-1);
            rightMotor.setPower(-1);
            delay(r, 0.4);
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }

        ru.reset();
        while (opModeIsActive() && ru.seconds() < 2) {
            beaconPresser.setPosition(0.6);
            delay(r, 0.4);
            beaconPresser.setPosition(0.2);
            delay(r, 0.4);
        }
        beaconPresser.setPosition(1);
    }

    public void delay(ElapsedTime runtime, double seconds) {
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < seconds) {}
    }
}
//sensor servo
//0.7 = out
//0.42 = part out