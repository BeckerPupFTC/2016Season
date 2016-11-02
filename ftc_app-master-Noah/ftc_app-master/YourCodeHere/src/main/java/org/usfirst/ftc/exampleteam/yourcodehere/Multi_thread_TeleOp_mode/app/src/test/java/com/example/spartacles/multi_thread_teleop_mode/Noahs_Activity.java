/* Note: We may be incorporating one or more touch sensors to the robot
 * to detect when the ball is on it's way up so that when it is close to
 * the launcher the launcher will start and then when it leaves it will stop.
 */

/* I am having trouble understanding this package thing and why import is
 * not working right, so we might have to cut and paste this code to run it.
 */
package com.example.spartacles.multi_thread_teleop_mode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
//import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
//import com.qualcomm.robotcore.hardware.ServoController;

/* You might have to declare this class abstract because run() calls
 * check(), which is not defined inside MyThreads. If all else fails,
 * we can just define an empty check() method in MyThreads and then
 * override the check() in the subclasses
 */
public class MyThreads implements Runnable {
    Thread thread;
    MyThreads(String name) {
        thread = new Thread(this, name);
    }
    public void begin() {
        start();
    }
    public void run() {
        while (true) {
            check();
        }
    }
}

public class LauncherThread extends MyThreads {
    public void check() {
        //shooter
        if (gamepad2.x) {
            try {
                float velocity;
                velocity = 1;
				/* Note: must set velocity to a predetermined constant or
				 * somehow use a stick to set velocity to the correct speed
				 * for a certain distance. I don't know if 1 is maximum
				 * power, but I'm just using that for now. If you're going to
				 * use the gamepad to control velocity, I'll need help.
				 */
                launcher.setPower(velocity);
                long motorTime = 500L; //Just a guess.
                dropBallIntoLauncher();
				/* Just giving it its own method because I don't know exactly
				 * how to code dropping the ball into the launcher.
				 */
                Thread.sleep(motorTime);
            } catch (InterruptedException exc) {}
        }
    }
    public void dropBallIntoLauncher() {
        /* Sounds like this will be nothing other than the conveyor belt moving,
         * so I may have to redo the launcher section. See comment at top.
         */
    }
}
//public class LauncherAngleThread extends MyThreads {public void check() {
//        float angle;
//        angle = gamepad2.left_stick_y;
//		/* Note: this angle variable probably needs to be scaled up
//		* or down in order to set the launcherAngle correctly.
//		*/
//      launcherAngle.setPosition(angle);
//    }
//}
public class ConveyorThread extends MyThreads {
    /* Note: may be taking out ConveyorThread because depending on whether or
     * not the conveyor is always on.
     */
    public void check() {
        //conveyor
        if (gamepad1.x) {
            conveyor.setPower(0.6);
        }
        if (gamepad1.b) {
            conveyor.setPower(0);
        }
    }
}
public class WheelsThread extends MyThreads {
    public void check() {
        //wheels
        float throttle = -gamepad1.left_stick_y;
        float direction = gamepad1.left_stick_x;
        float right = throttle - direction;
        float left = throttle + direction;
        wheelR.setPower(right);
        wheelL.setPower(left);
    }
}

public class Robot_runner extends OpMode {

    /* Does this code need a main method to call init and then repeat loop?
     * Also, do I need additional code to initialize motor and servo controllers?
     */
    DcMotorController wheelController;
    DcMotorController otherController;
    DcMotor wheelR;
    DcMotor wheelL;
    DcMotor conveyor;  // Will the intake brush also be wired to this?
    DcMotor launcher;
//    Servo launcherAngle;
    DcMotor ballDropper;

    @Override
    public void init() {

        wheelController = hardwareMap.dcMotorController.get("motor_controller_1");
        wheelR = hardwareMap.dcMotor.get("motor_1");
        wheelL = hardwareMap.dcMotor.get("motor_2");

        otherController = hardwareMap.dcMotorController.get("motor_controller_2");
        conveyor = hardwareMap.dcMotor.get("motor_3");
        launcher = hardwareMap.dcMotor.get("motor_4");
        //launcherAngle = hardwareMap.servo.get("servo_1");
        //ballDropper = hardwareMap.dcMotor.get("motor_5");

        LauncherThread launcher_check = new LauncherThread("launcher checker");
        LauncherAngleThread launcher_angle_check = new LauncherAngleThread("launcher angle checker");
        ConveyorThread conveyor_check = new ConveyorThread("conveyor checker");
        WheelsThread wheels_check = new WheelsThread("wheel checker");

    }

    @Override
    public void loop () {
        launcher_check.begin();
        launcher_angle_check.begin();
        conveyor_check.begin();
        wheels_check.begin();
        while(true){}
    }

    @Override
    public void stop() {
        //What's supposed to be here?
    }
}