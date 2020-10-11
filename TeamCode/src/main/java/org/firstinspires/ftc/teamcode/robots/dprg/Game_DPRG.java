package org.firstinspires.ftc.teamcode.robots.dprg;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * This file contains the code for Iron Reign's main OpMode, used for both TeleOp and Autonomous.
 */
@Disabled
@TeleOp(name = "Game_DPRG", group = "Challenge")  // @Autonomous(...) is the other common choice
public class Game_DPRG extends LinearOpMode {

    Pose_DPRG robot = new Pose_DPRG();
    double pwrFwd = 0;
    double pwrRot = 0;

    long lastLoop = 0;
    @Override
    public void runOpMode() throws InterruptedException {



        robot.init(this.hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();



        while (!isStarted()) {    // Wait for the game to start (driver presses PLAY)
            synchronized (this) {
                try {
                    this.wait();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return;
                }
            }

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }





        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.update();
            joystickDrive();
            while (System.currentTimeMillis() < lastLoop+40);
            lastLoop = System.currentTimeMillis();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }




        robot.stopAll();
    }



    public void joystickDrive() {

        pwrFwd = gamepad1.left_stick_y;
        pwrRot = .75 * gamepad1.right_stick_x;

        robot.driveTank(pwrFwd,pwrRot);


    }

}