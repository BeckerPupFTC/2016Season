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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving up to a line and then stopping.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code shows using two different light sensors:
 *   The Primary sensor shown in this code is a legacy NXT Light sensor (called "sensor_light")
 *   Alternative "commented out" code uses a MR Optical Distance Sensor (called "sensor_ods")
 *   instead of the LEGO sensor.  Chose to use one sensor or the other.
 *
 *   Setting the correct WHITE_THRESHOLD value is key to stopping correctly.
 *   This should be set half way between the light and dark values.
 *   These values can be read on the screen once the OpMode has been INIT, but before it is STARTED.
 *   Move the sensor on and off the white line and not the min and max readings.
 *   Edit this code to make WHITE_THRESHOLD half way between the min and max.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="AwesomeAuto", group="Pushbot")
//@Disabled
public class AwesomeAuto extends LinearOpMode {

    /* Declare OpMode members. */
    DcMotor rightMotor;
    DcMotor leftMotor;
    DcMotor intake;
    DcMotor launcher;
    Servo intake_servo;
    LightSensor lightSensor;      // Primary LEGO Light sensor,
    // OpticalDistanceSensor   lightSensor;   // Alternative MR ODS sensor
    ColorSensor colorSensor;
    TouchSensor touchSensor;

    private ElapsedTime runtime = new ElapsedTime();

    static final double WHITE_THRESHOLD = 0.2;  // spans between 0.1 - 0.5 from dark to light

    static final double     COUNTS_PER_MOTOR_REV      = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION      = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES     = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH           = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DIST_BETWEEN_BACK_WHEELS  = 16.5;     //May need to change
    static final double     DEGREE_TURN_INCHES = DIST_BETWEEN_BACK_WHEELS * 0.0087266;
    static final double     DEGREE_TURN_UNITS  = DEGREE_TURN_INCHES * COUNTS_PER_INCH;

    static final double     COLOR_THRESHOLD           = 128;     //May need to change

    @Override
    public void runOpMode() {

        rightMotor = hardwareMap.dcMotor.get("wheelR");
        leftMotor = hardwareMap.dcMotor.get("wheelL");
        intake = hardwareMap.dcMotor.get("launcher");
        launcher = hardwareMap.dcMotor.get("intake");
        intake_servo = hardwareMap.servo.get("servo_1");
        //beacon_presser = hardwareMap.servo.get("servo_2");
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);//This motor is pointing the wrong direction

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // get a reference to our Light Sensor object.
        lightSensor = hardwareMap.lightSensor.get("sensor_light");                // Primary LEGO Light Sensor
        //  lightSensor = hardwareMap.opticalDistanceSensor.get("sensor_ods");  // Alternative MR ODS sensor.

        // turn on LED of light sensor.
        lightSensor.enableLed(true);

        // get a reference to our ColorSensor object.
        colorSensor = hardwareMap.colorSensor.get("sensor_color");

        // Set the LED in the beginning
        colorSensor.enableLed(false);

        touchSensor = hardwareMap.touchSensor.get("sensor_touch");

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        // Abort this loop is started or stopped.
        while (!(isStarted() || isStopRequested())) {

            // Display the light level while we are waiting to start
            telemetry.addData("Light Level", lightSensor.getLightDetected());
            telemetry.update();
            idle();
        }

        driveTillLine();    //drive to line
        if(colorSensor.red() > COLOR_THRESHOLD) {
            encoderDrive(4, 1.2);
        }
        encoderDrive(90);
        sleep(1000);  //turn right

        driveTillTouch();    //drive to the wall

        encoderDrive(3, 1);
        encoderDrive(-3, 1);
        encoderDrive(3, 1);
        encoderDrive(-3, 1);    //ram the beacon

        encoderDrive(12, 4);    //go back up the line

        encoderDrive(-90);
        sleep(1000);    //turn left

        driveTillLine();    // drive to line

        if(colorSensor.red() > COLOR_THRESHOLD) {
            encoderDrive(4,1.2);
        }

        encoderDrive(90);
        sleep(1000);    //turn right

        driveTillTouch();    //drive to wall

        encoderDrive(3, 1);
        encoderDrive(1-3, 1);
        encoderDrive(3, 1);
        encoderDrive(-3, 1);    //ram the beacon
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double rightInches, double timeoutS) {
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newRightTarget = rightMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION

            // reset the timeout time and start motion.
            runtime.reset();
            if(rightInches<0) {
                leftMotor.setPower(-1);
            } else {
                leftMotor.setPower(1);
            }
            rightMotor.setPower(1);

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && rightMotor.isBusy()) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d", newRightTarget);
                telemetry.addData("Path2", "Running at %7d",
                        rightMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftMotor.setPower(0);
            rightMotor.setPower(0);

        }
    }
    public void encoderDrive(double degrees) {
        int newRightTarget;
        if(opModeIsActive()) {
            newRightTarget = rightMotor.getCurrentPosition() + (int) (DEGREE_TURN_UNITS*degrees);

            rightMotor.setTargetPosition(newRightTarget);
            rightMotor.setPower(1);
            if (degrees<0) {
                leftMotor.setPower(-1);
            } else {
                rightMotor.setPower(1);
            }
            while (opModeIsActive() && rightMotor.isBusy()) {
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d", newRightTarget);
                telemetry.addData("Path2", "Running at %7d", rightMotor.getCurrentPosition());
                telemetry.update();
            }
            // Stop all motion;
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }
    }
    public void driveTillTouch() {
        int newRightTarget;
        if(opModeIsActive()) {
            newRightTarget = rightMotor.getCurrentPosition() - (int) (24*COUNTS_PER_INCH);
            while (opModeIsActive() &&
                    !touchSensor.isPressed()) {
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d", newRightTarget);
                telemetry.addData("Path2", "Running at %7d",
                        rightMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            rightMotor.setTargetPosition(rightMotor.getCurrentPosition());
        }
    }
    public void driveTillLine() {
        // Start the robot moving forward, and then begin looking for a white line.
        leftMotor.setPower(1);
        rightMotor.setPower(1);

        // run until the white line is seen OR the driver presses STOP;
        while (opModeIsActive() && (lightSensor.getLightDetected() < WHITE_THRESHOLD)) {

            // Display the light level while we are looking for the line
            telemetry.addData("Light Level", lightSensor.getLightDetected());
            telemetry.update();
        }

        // Stop all motors
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
}