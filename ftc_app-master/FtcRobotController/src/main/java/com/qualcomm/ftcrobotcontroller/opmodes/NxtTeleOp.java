

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.ServoController;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class NxtTeleOp extends OpMode {

  DcMotor wheelR;
  DcMotor wheelL;
  DcMotor conveyor;
  Servo shooterR;
  Servo shooterL;

  @Override
  public void init() {
    wheelR = hardwareMap.dcMotor.get("motor_1");
    wheelL = hardwareMap.dcMotor.get("motor_2");
    conveyor = hardwareMap.dcMotor.get("motor_3");
    shooterR = hardwareMap.servo.get("servo_1");
    shooterL = hardwareMap.servo.get("servo_2");
  }

  @Override
  public void loop () {
    //wheels
    float throttle = -gamepad1.left_stick_y;
    float direction = gamepad1.left_stick_x;
    float right = throttle - direction;
    float left = throttle + direction;
    wheelR.setPower(right);
    wheelL.setPower(left);

    //conveyor
    conveyor.setPower(0.6);

    //shooter
    if(gamepad1.x) {
      shooterL.setPosition(2);
      shooterR.setPosition(2);
    }
  }

@Override
public void stop() {

  }
}
