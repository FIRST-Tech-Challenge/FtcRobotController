package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class LongRedAuto extends LinearOpMode {




    @Override
    public void runOpMode() throws InterruptedException {
        boolean isDone = false;
        int idNumber = 6;
        Robot robot = new Robot(hardwareMap, this, telemetry);

        robot.setUpAprilTags();

        waitForStart();


        robot.detectAndMoveToMarker();
        robot.waitFor(0.75);

        robot.straightBlocking(15, true);
        robot.waitFor(0.1);
        robot.setHeading(0);
        robot.waitFor(0.1);
        robot.mecanumBlocking(20, false);
        robot.waitFor(0.1);
        robot.setHeading(0);
        robot.waitFor(0.1);
        robot.straightBlocking(38, true);
        robot.waitFor(0.1);
        robot.setHeading(-90);
        robot.waitFor(0.1);
        robot.straightBlocking(88, true);
        robot.waitFor(0.1);
        robot.setHeading(-90);
        robot.waitFor(0.1);
        robot.mecanumBlocking(25, true);
        //TODO: APRILTAG GOES HERE !!!!!!
        while (opModeIsActive() && !isDone) {


            if (!robot.moveRelativeToAprilTagX(0, idNumber)) {
                isDone = robot.moveRelativeToAprilTagX(0, idNumber);
            }

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


        robot.waitFor(2);
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


    }
}
