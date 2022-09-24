package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.bots.NewDistanceSensorBot;

@Autonomous(name="Auto 3", group="Autos")
@Disabled
public class Auto3 extends LinearOpMode {

    protected NewDistanceSensorBot robot = new NewDistanceSensorBot(this);

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.isRepeating = true;
        waitForStart();
//        int[] pos;
//        pos = robot.detect();
        if (true) {
            // STATUS_EXTENDING
            robot.goToIntakePosition(3);
            // ?? Rotating turrent ?
            robot.setExtension(robot.maxExtension);
            robot.waitOnExtension(robot.maxExtension);
            robot.sleep(100);
            // CONDITION : wait for extension + 100ms

            // STATUS_RELEASING_RETRACTING_DRIVING
            robot.goToFlipperPosition(0);
            robot.extensionCheckpoints[3] = true;
            robot.setExtension(robot.minExtension);
            robot.setRotationPosition(robot.rotationCenter);

            robot.intakeFast = false;

            robot.driveByGyroWithEncodersVertical(robot.DIRECTION_FORWARD, 35000, false, 500, 0);
            robot.goToAngle(0, 0.16);
            robot.sleep(300);
            // CONDITION : wait for driving + 300ms

            // STATUS_INTAKING
            //robot.autoGrabFreight(0.2, 0);
            // CONDITION : distance sensor detected freight
            // STATUS_DRIVING_OUT_WAREHOUSE

            robot.goToAngle(0, 0.16);
            // CONDITION : wait for driving
            // STATUS_EXTENDING
            robot.waitOnExtension(robot.maxExtension);
            robot.sleep(1000);
            // CONDITION : wait for extension + 1000ms


            // STATUS_RELEASING_RETRACTING_DRIVING
            robot.goToFlipperPosition(0);
            robot.extensionCheckpoints[3] = true;
            robot.setExtension(robot.minExtension);
            robot.setRotationPosition(robot.rotationCenter);

            robot.driveByGyroWithEncodersVertical(robot.DIRECTION_FORWARD, 45000, false, 500, 0);
            robot.goToAngle(0, 0.16);
            // CONDITION : wait for driving

            robot.sleep(10000);
            //robot.driveByGyroWithEncodersVertical(robot.DIRECTION_BACKWARD, 10000, 0.6, false, false, 200);
        } else {

        }
    }
}
