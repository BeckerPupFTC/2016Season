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

@Autonomous(name="ServoTest", group="Beacon")
//@Disabled
public class ServoTest extends LinearOpMode {

    /* Declare OpMode members. */
    DcMotor rightMotor;
    DcMotor leftMotor;
    DcMotor intake;
    DcMotor launcher;
    Servo intake_servo;
    Servo beacon_presser;
    Servo sensorServo;
    LightSensor legoLightSensor;      // Primary LEGO Light sensor
    OpticalDistanceSensor   lightSensor;   // Alternative MR ODS sensor
    UltrasonicSensor rangeSensor;
    UltrasonicSensor backSensor;

    LegacyModule board;

    ElapsedTime r = new ElapsedTime();
    ElapsedTime ru = new ElapsedTime();

    static final double     WHITE_THRESHOLD = 0.2;  // spans between 0.1 - 0.5 from dark to light

    @Override
    public void runOpMode() {

        rightMotor = hardwareMap.dcMotor.get("wheelR");
        leftMotor = hardwareMap.dcMotor.get("wheelL");
        intake = hardwareMap.dcMotor.get("launcher");
        launcher = hardwareMap.dcMotor.get("intake");
        intake_servo = hardwareMap.servo.get("servo_1");
        beacon_presser = hardwareMap.servo.get("servo_2");
        sensorServo = hardwareMap.servo.get("servo_3");
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);//This motor is pointing the wrong direction

        backSensor = hardwareMap.ultrasonicSensor.get("back sensor");
        board = hardwareMap.legacyModule.get("Legacy Module 1");
        board.enable9v(4, true);

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // get a reference to our Light Sensor object.
        legoLightSensor = hardwareMap.lightSensor.get("sensor_light");                // Primary LEGO Light Sensor
        lightSensor = hardwareMap.opticalDistanceSensor.get("sensor_ods");  // Alternative MR ODS sensor.

        // turn on LED of light sensor.
        lightSensor.enableLed(true);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        // Abort this loop is started or stopped.
        waitForStart();

        // Start the robot moving forward, and then begin looking for a white line
        beacon_presser.setPosition(1);
        delay(r, 1);
        beacon_presser.setPosition(0);
        delay(r, 1);
        beacon_presser.setPosition(0.5);
        delay(r, 1);
        beacon_presser.setPosition(0.25);
        delay(r, 1);
        beacon_presser.setPosition(0.75);
        delay(r, 1);
        beacon_presser.setPosition(1);
        delay(r, 1);
        beacon_presser.setPosition(0);
    }

    public void delay(ElapsedTime runtime, double seconds) {
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < seconds) {}
    }
}