package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous
public class LongRedAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        int idNumber = 0;

        //robot, dt motors, vision processing setup
        Robot robot = new Robot(hardwareMap, this, telemetry);
        //TODO uncomment
        robot.setUpDrivetrainMotors();
        //robot.initVisionProcessing();
        Servo holderClamp = hardwareMap.servo.get("holderClamp");

        robot.resetLinearSlideEncoder();

        double armPos = 0.594; //down position
        double holderClampPos = 0.5;

        double remainingDistanceLow = 100 - robot.lsFront.getCurrentPosition();
        double remainingDistanceZero = -robot.lsFront.getCurrentPosition();

        //vision processing stuff
        /*
        VisionPortal visionPortal = robot.getVisionPortal();

        MarkerProcessor markerProcessor = robot.getMarkerProcessor();
        AprilTagProcessor aprilTagProcessor = robot.getAprilTagProcessor();

        MarkerDetector.MARKER_POSITION position;
        */

        sleep(3000);
        waitForStart();



        while (opModeIsActive()) {
            robot.autoOuttake();
            //detectmarkerposition() code commented below
            /*
            //detect marker position
            position = markerProcessor.getPosition();

            while (position == MarkerDetector.MARKER_POSITION.UNDETECTED) {
                Log.d("vision", "undetected marker, keep looking");
                position = markerProcessor.getPosition();
            }

            //print position
            Log.d("vision", "detected position: " + position);

            //save marker position, apriltag position
            robot.setMarkerPos(position);
            robot.setWantedAprTagId(position, true);
            */

            //TODO Uncomment
           /* robot.detectMarkerPosition();
            robot.moveToMarker();*/

            //sleep(1000);

            //move to backdrop from spike marks
            //TODO uncomment
            /*robot.setHeading(0, 0.75);
            sleep(100);
            robot.mecanumBlocking(20, true, 0.5);
            sleep(100);
            robot.setHeading(0, 0.75);
            sleep(100);
            robot.straightBlocking(35, false, 0.7);
            sleep(100);
            robot.setHeading(-90, 0.75);
            sleep(100);
            robot.straightBlocking(90, false, 0.7);
            sleep(100);
            robot.setHeading(-90, 0.5);
            sleep(100);
            robot.mecanumBlocking(28, false, 0.5);
            sleep(100);
            robot.setHeading(-90, 0.75);

            sleep(2000);

            robot.moveToBoard();
            sleep(100);
            robot.setHeading(-90, 0.75);
*/
          /*  double targetDistanceInTicks = 350;

            remainingDistanceLow = targetDistanceInTicks - robot.lsFront.getCurrentPosition();
            remainingDistanceZero = -robot.lsFront.getCurrentPosition();

            telemetry.addData("linear slide pos in ticks", robot.lsFront.getCurrentPosition());

            if (robot.lsFront.getCurrentPosition() > -10) {
                robot.lsFront.setPower(remainingDistanceLow * -0.002);
                robot.lsBack.setPower(remainingDistanceLow * -0.002);
            } else if (robot.lsFront.getCurrentPosition() < -targetDistanceInTicks) {
                robot.lsFront.setPower(0);
                robot.lsBack.setPower(0);
            }



            armPos = 0.845; //up (outtake) position
            robot.arm.setPosition(armPos);

            holderClampPos = 0.3;
            holderClamp.setPosition(holderClampPos);

            armPos = 0.594; //down (inttake) position
            robot.arm.setPosition(armPos);
*/
            /*if (robot.lsFront.getCurrentPosition() < 10) {
                robot.lsFront.setPower(remainingDistanceZero * 0.002);
                robot.lsBack.setPower(remainingDistanceZero * 0.002);
            } else {
                robot.lsFront.setPower(0);
                robot.lsBack.setPower(0);
            }*/

            telemetry.update();
            //break;

            //movetoboard() code commented below
            break;
        }

    }
}

