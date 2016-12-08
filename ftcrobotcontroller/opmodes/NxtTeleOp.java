package com.qualcomm.ftcrobotcontroller.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
//import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
//import com.qualcomm.robotcore.hardware.ServoController;
abstract public class MyThreads implements Runnable
{
  Thread thread;

  MyThreads(String name) {
    thread = new Thread(this, name);
  }
  public void begin()
  {
    start();
  }
  public void run()
  {
    while (opModeIsActive)
    {
      check();
      idle();
    }
  }
  abstract public void check();
}

public class LauncherThread extends MyThreads
{
  public void check()
  {
    //shooter
    if (gamepad2.x)
    {
      //float velocity;
      //velocity = 1;
      /* Note: must set velocity to a predetermined constant or
       * somehow use a stick to set velocity to the correct speed
       * for a certain distance. I don't know if 1 is maximum
       * power, but I'm just using that for now. If you're going to
       * use the gamepad to control velocity, I'll need help.
       */
      //launcher.setPower(velocity);
      //long motorTime = 500L; //Just a guess.
      dropBallIntoLauncher();
      /* Just giving it its own method because I don't know exactly
       * how to code dropping the ball into the launcher.
       */
      //Thread.sleep(motorTime);
    }
  }
  public void dropBallIntoLauncher()
  {
    /* Sounds like this will be nothing other than the conveyor belt moving,
     * so I may have to redo the launcher section. See comment at top.
     */
    launcher.setPower(1);
    rotor.setTargetPosition(rotor.getPosition());
    rotor.setPower(0.5);
    while (rotor.isBusy()){}
    rotor.setPower(0);
    launcher.setPower(0);
  }
}
//public class LauncherAngleThread extends MyThreads /
//{
//   public void check()
//   {
//      float angle;
//      angle = gamepad2.left_stick_y;
//      /* Note: this angle variable probably needs to be scaled up
//      * or down in order to set the launcherAngle correctly.
//      */
//      launcherAngle.setPosition(angle);
//   }
   //}

//public class ConveyorThread extends MyThreads
// {
//  /* Note: may be taking out ConveyorThread because depending on whether or
//  * not the conveyor is always on.
//   */
//  public void check()
//  {
//    //conveyor
//    if (gamepad1.x)
//    {
//      conveyor.setPower(0.6);
//    }
//    if (gamepad1.b)
//    {
//      conveyor.setPower(0);
//    }
//  }
//}
public class WheelsThread extends MyThreads
{
  public void check()
  {
    //wheels
    float throttle = -gamepad1.left_stick_y;
    float direction = gamepad1.left_stick_x;
    float right = throttle - direction;
    float left = throttle + direction;
    wheelR.setPower(right);
    wheelL.setPower(left);
  }
}

@TeleOp(name = "remote control code")
public class Robot_runner extends LinearOpMode
{
  DcMotorController wheelController;
  DcMotorController otherController;
  DcMotor wheelR;
  DcMotor wheelL;
  //DcMotor conveyor;  // Will the intake brush also be wired to this?
  DcMotor launcher;
  //Servo launcherAngle;
  DcMotor rotor;
  rotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
  @Override
  public void runOpMode()
  {
    wheelController = hardwareMap.dcMotorController.get("motor controller 1");
    wheelR = hardwareMap.dcMotor.get("motor 1");
    wheelL = hardwareMap.dcMotor.get("motor 2")

    otherController = hardwareMap.dcMotorController.get("motor controller 2");
    //conveyor = hardwareMap.dcMotor.get("motor 3");
    rotor = hardwareMap.dcMotor.get("motor 3");
    launcher = hardwareMap.dcMotor.get("motor 4");

    //launcherAngle = hardwareMap.servo.get("servo 1");

    LauncherThread launcher_check = new LauncherThread("launcher checker");
    LauncherAngleThread launcher_angle_check = new LauncherAngleThread("launcher angle checker");
    ConveyorThread conveyor_check = new ConveyorThread("conveyor checker");
    WheelsThread wheels_check = new WheelsThread("wheel checker");

    waitForStart();

    conveyor.setPower(0.6);

    launcher_check.begin();
    //launcher_angle_check.begin();
    conveyor_check.begin();
    wheels_check.begin();
    try
    {
      launcher_check.thread.join();
      conveyor_check.thread.join();
      wheels_check.thread.join();
      //launcher_angle_check.join();
    }
    catch (InterruptedException e)
    {
      telemetry.addData("For some reason one of the calls to join failed.");
      telemetry.addData("This might mean nothing, but then again, it might mean that");
      telemetry.addData("The program will stop suddenly and unexpectedly in the middle");
      telemetry.addData("of a match, requiring you to restart and thus consuming valuable time.");
      telemetry.addData("Also, the robot may not function correctly due to parts being in a");
      telemetry.addData("nonzero position.\n");
      telemetry.addData("Bummer, huh?");
      telemetry.update;
    }
  }
}