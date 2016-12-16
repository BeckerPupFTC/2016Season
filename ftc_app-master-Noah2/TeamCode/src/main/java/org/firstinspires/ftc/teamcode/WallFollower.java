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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

/**
 * {@link WallFollower} illustrates how to use the Modern Robotics
 * Range Sensor.
 *
 * The op mode assumes that the range sensor is configured with a name of "sensor_range".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see <a href="http://modernroboticsinc.com/range-sensor">MR Range Sensor</a>
 */
@Autonomous(name = "WallFollower", group = "Sensor")
//@Disabled   // comment out or remove this line to enable this opmode
public class WallFollower extends LinearOpMode {

    UltrasonicSensor rangeSensor;
    LegacyModule board;

    DcMotor wheelL;
    DcMotor wheelR;
    Servo sensorServo;
    Servo beaconPresser;

    @Override public void runOpMode() {

        rangeSensor = hardwareMap.ultrasonicSensor.get("sensor_ultrasonic");
        board = hardwareMap.legacyModule.get("Legacy Module 1");
        board.enable9v(4, true);

        sensorServo = hardwareMap.servo.get("servo_3");
        beaconPresser = hardwareMap.servo.get("servo_2");

        wheelL = hardwareMap.dcMotor.get("wheelL");
        wheelR = hardwareMap.dcMotor.get("wheelR");
        wheelL.setDirection(DcMotorSimple.Direction.REVERSE);
        sensorServo.setPosition(0.58);
        beaconPresser.setPosition(0.94);

        // wait for the start button to be pressed
        while (!(isStarted() || isStopRequested())) {

            // Display the light level while we are waiting to start
            telemetry.addData("distance: ", rangeSensor.getUltrasonicLevel());
            telemetry.update();
            idle();
        }

        wheelL.setPower(0.8);
        wheelR.setPower(0.88);

        double distanceThen;
        double distanceNow = 0;
        double level = 0.2;

        while(opModeIsActive()) {
            distanceThen = distanceNow;
            distanceNow = rangeSensor.getUltrasonicLevel();
            telemetry.addData("distance: ", distanceNow);
            telemetry.addData("previously:", distanceThen);
            telemetry.addData("power: ", wheelR.getPower());
            telemetry.update();
            if(distanceNow < 17) {
                if (distanceNow <= distanceThen) {
                    level = level + 0.003;
                } else {
                    level = level - 0.002;
                }
            } else if(distanceNow > 18) {
                if (distanceNow >= distanceThen) {
                    level = level - 0.003;
                } else {
                    level = level + 0.002;
                }
            }
            wheelR.setPower(level);
            sleep(6);
        }
    }
}