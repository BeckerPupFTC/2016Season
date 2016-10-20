/* I am having trouble understanding this package thing and why import is
 * not working right, so we might have to cut and paste this code to run it.
 */
package com.example.spartacles.robot_code;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.ServoController;


public class Noahs_Activity extends OpMode {
    /* Does anybody know if the phones we'll be using have multi-core
     * processors and if so, are we able to divide all of these gamepad
     * position checks among the cores and if so, how?
     */
    /* On second thoughts, that may be a waste of time.
     * Never mind.
     */
    /* Does this code need a main method to call init and then repeat loop?
     * Also, do I need additional code to initialize motor and servo controllers?
     */
    DcMotor wheelR;
    DcMotor wheelL;
    DcMotor conveyor;  // Will the intake brush also be wired to this?
    DcMotor launcher;
    Servo launcherAngle;
    DcMotor ballDropper;
    @Override
    public void init() {
        /* Do the continuous rotation servos need to set up some way
         * other than this? I can't use  .setPosition  because these
         * servos are basically dc motors, but it will be connected to a
         * servo controller, so I don't know what to do with these. I
         * have called these dc motors for now, but that won't work either.
         */
        wheelR = hardwareMap.dcMotor.get("motor_1");
        wheelL = hardwareMap.dcMotor.get("motor_2");
        conveyor = hardwareMap.dcMotor.get("motor_3");
        launcher = hardwareMap.dcMotor.get("motor_4");
        launcherAngle = hardwareMap.servo.get("servo_1");
        ballDropper = hardwareMap.dcMotor.get("motor_5");

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
        if (gamepad1.x) {
            conveyor.setPower(0.6);
        }

        //shooter

        float angle;
        angle = gamepad2.left_stick_y;
        /* Note: this angle variable probably needs to be scaled up
         * or down in order to set the launcherAngle correctly.
         */
        float velocity;
        velocity = 1;
        /* Note: must set velocity to a predetermined constant or
         * somehow use a stick to set velocity to the correct speed
         * for a certain distance. I don't know if 1 is maximum
         * power, but I'm just using that for now. If you're going to
         * use the gamepad to control velocity, I'll need help.
         */
        launcherAngle.setPosition(angle);
        launcher.setPower(velocity);
        dropBallIntoLauncher();

        }

    public void dropBallIntoLauncher() {
        //Don't know how this will work. Will try anyway.
        long motorTime = 500L; //Just a guess.
        float motorPower = 0.5 //Also just a guess.
        ballDropper.setPower(motorPower);
        /* I'm not sure if all this Thread.sleep
         * is what I'm supposed to do. I need more help.
         */
        Thread.sleep(motorTime);
        }
    @Override
    public void stop() {

        }
}