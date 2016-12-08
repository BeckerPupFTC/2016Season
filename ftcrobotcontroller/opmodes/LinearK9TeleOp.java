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
/* Code contains a couple of important comments. Please look through them.
 * Hope this works :)
 */
package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.DcMotorController;

/**
 * Linear Tele Op Mode
 * <p>
 * Enables control of the robot via the gamepad.
 */

public class LinearK9TeleOp extends OpMode {
  @Override
  public void init() {
    //initialization
    DcMotorController wheelController;
    DcMotor motorRight;
    DcMotor motorLeft;

    DcMotorController launcherController;
    DcMotor intake;
    DcMotor launcher;

    ServoController servoController;
    Servo intakeServo;

    wheel_controller = hardwareMap.dcMotorController.get("wheel controller");
    motorLeft = hardwareMap.dcMotor.get("left wheel");
    motorRight = hardwareMap.dcMotor.get("right wheel");

    launcherController = hardwareMap.dcMotorController.get("launcher controller");
    intake = hardwareMap.dcMotor.get("intake");

    servoController = hardwareMap.servoController.get("servo controller");
    intakeServo = hardwareMap.servo.get("servo");
  }
}

  @Override
  public void loop() {
    //intake
    if (gamepad1.x) {
      intake.setPower(1);
    }
    if (gamepad1.b) {
      intake.setPower(-1);
    }
    if (gamepad1.a || gamepad1.y) {
      intake.setPower(0);
    }
    //wheels
    float throttle = -gamepad1.left_stick_y;
    float direction = gamepad1.left_stick_x;
    float right = throttle - direction;
    float left = throttle + direction;
    /* this is a one-joystick control scheme.
	 * for a two wheel drive differential,
	 * comment out the previous code and
	 * uncomment the following code.
	 */
    //float left = gamepad1.left_stick_y;
    //float right = gamepad1.right_stick_y;
    wheelR.setPower(right);
    wheelL.setPower(left);
    //launcher
    if (gamepad1.dpad_up) {
      intake.setPower(0);
      motorLeft.setPower(0);
      motorRight.setPower(0);
      intakeServo.setPosition(1);//might need to be changed to 0
      launcher.setPower(1);//might need to be changed to -1
      try {
        Thread.sleep(3000);
      } catch (Exception e) {
      }
      intakeServo.setPosition(0.5);
      launcher.setPower(0);
    }
  }
}