package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class LongRedAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        boolean isDone = false;
        int idNumber = 1;
        Robot robot = new Robot(hardwareMap, this, telemetry);
        robot.setUpDrivetrainMotors();
        Log.d("vision", "pre set up vision");
        robot.setUpVisionProcessing();

        waitForStart();
        Log.d("vision", "just ran");


        robot.detectAndMoveToMarker();
        Log.d("vision", "after kaitlyns big mess");
        robot.waitFor(0.75);
        /*
        robot.straightBlocking(15, false);
        robot.waitFor(0.1);
        robot.setHeading(0);
        robot.waitFor(0.1);
        robot.mecanumBlocking(20, true);
        robot.waitFor(0.1);
        robot.setHeading(0);
        robot.waitFor(0.1);
        robot.straightBlocking(38, false);
        robot.waitFor(0.1);
        robot.setHeading(-90);
        robot.waitFor(0.1);
        robot.straightBlocking(88, false);
        robot.waitFor(0.1);
        robot.setHeading(-90);
        robot.waitFor(0.1);
        robot.mecanumBlocking(28, false);*/

        //TODO: APRILTAG GOES HERE !!!!!!

        while (opModeIsActive() && !isDone) {


            if (!robot.moveRelativeToAprilTagX(0, idNumber)) {
                isDone = robot.moveRelativeToAprilTagX(0, idNumber);
            }
            Log.d("vision", "post ethens big mess");
            telemetry.addData("x", isDone);
            //robot.moveRelativeToAprilTag(10);

            telemetry.update();
        }
        telemetry.addLine("got out");
        isDone = false;
        while (opModeIsActive() && !isDone) {


            if (!robot.moveRelativeToAprilTagRange(10, idNumber)) {
                isDone = robot.moveRelativeToAprilTagRange(10, idNumber);
            }

            telemetry.addData("range", isDone);
            //robot.moveRelativeToAprilTag(10);

            telemetry.update();
        }


        /*robot.waitFor(2);
        robot.setHeading(0);
        robot.waitFor(0.1);
        robot.straightBlocking(24, true);
        robot.waitFor(0.1);
        robot.setHeading(90);
        robot.waitFor(0.1);
        robot.straightBlocking(84, true);
        robot.waitFor(0.1);
        robot.setHeading(-90);
        robot.waitFor(0.1);
        robot.straightBlocking(84, true);
        robot.waitFor(0.1);
        robot.setHeading(-90);
        robot.waitFor(0.1);
        robot.mecanumBlocking(25, true);
        robot.waitFor(0.1);
*/

    }
}
