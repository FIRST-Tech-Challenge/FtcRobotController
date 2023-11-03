package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class LongBlueAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        int idNumber = 0;

        //robot, dt motors, vision processing setup
        Robot robot = new Robot(hardwareMap, this, telemetry);
        //TODO uncomment
        robot.setUpDrivetrainMotors();
        robot.initVisionProcessingBlue();

        /*
        Servo holderClamp = hardwareMap.servo.get("holderClamp");

        robot.resetLinearSlideEncoder();

        double armPos = 0.594; //down position
        double holderClampPos = 0.5;

        double remainingDistanceLow = 100 - robot.lsFront.getCurrentPosition();
        double remainingDistanceZero = -robot.lsFront.getCurrentPosition();
        */

        waitForStart();

        while (opModeIsActive()) {

            /*
            robot.autoOuttake();
            Thread.sleep(10000);
            */

            //TODO Uncomment
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

            //telemetry.update();


            //enable THIS to test longredauto
            robot.detectMarkerPositionBlue();
            robot.longRedMoveToBoard();
            //robot.alignToBoard();
            //robot.setHeading(-90, 0.75);

            break;
        }
    }
}

