package org.firstinspires.ftc.teamcode.toolkit.background;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.UpliftRobot;
import org.firstinspires.ftc.teamcode.toolkit.misc.Utils;
@Disabled
public class AutoTimeout implements Runnable {

    Thread t;
    UpliftRobot robot;
    LinearOpMode opMode;
    int loopCounter = 0;

    public AutoTimeout(UpliftRobot robot) {
        t = new Thread(this);
        this.robot = robot;
        this.opMode = robot.opMode;
        t.start();
    }

    @Override
    public void run() {
        opMode.waitForStart();
        while(t != null && opMode.opModeIsActive() && !opMode.isStopRequested()) {
            double initialX = robot.worldX;
            double initialY = robot.worldY;
            double initialAngle = robot.rawAngle;
            Utils.sleep(1000);
            double deltaX = Math.abs(robot.worldX - initialX);
            double deltaY = Math.abs(robot.worldY - initialY);
            double deltaAngle = Math.abs(robot.rawAngle - initialAngle);
            // if robot has not moved much, but at least one of the drive motors is receiving power...
            if ((deltaX < 0.5 && deltaY < 0.5 && deltaAngle < 1) && (robot.leftFront.getPower() != 0 || robot.leftBack.getPower() != 0 || robot.rightFront.getPower() != 0 || robot.rightBack.getPower() != 0)) {
                // increment the loop counter
                loopCounter++;
                if (loopCounter >= 5) {
                    // since the robot has stalled for at least five seconds (loops), timeout (cancel)
                    robot.opMode.stop();
                }
            } else {
                // reset the loop counter
                loopCounter = 0;
            }
        }
    }

    public void stop() {
        t = null;
    }
}
