package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


public class AutoRed extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, this, telemetry);
        robot.setUpDrivetrainMotors();
//        robot.setUpArmMotor();

        robot.initVisionProcessingRed();
        waitForStart();


//        double mecanumDistance = -711.2;
//        double forwardDistance = 711.2;
//        double forwardDistance2 = -482.6;
//        double forwardDistance3 = 889;
//        double forwardDistance4 = 2438.4;
//        double turnDegrees1 = 90;
//        double mecanumDistance2 = 609.6;
//
//
////        double armPower = robot.calculateArmPower(90);
//        double[] forwardPower = robot.calculateDrivetrainPower(forwardDistance);
//        double[] mecanumPower = robot.calculateMecanumPower(mecanumDistance);
//        double[] forwardPower2 = robot.calculateDrivetrainPower(forwardDistance2);
//        double[] forwardPower3 = robot.calculateDrivetrainPower(forwardDistance3);
//        double[] forwardPower4 = robot.calculateDrivetrainPower(forwardDistance4);
//        double[] mecanumPower2 = robot.calculateMecanumPower(mecanumDistance2);
//
//
//        boolean doneWithMecanum = false;
//        boolean doneWithForward = false;
//        boolean doneWithForward2 = false;
//        boolean doneWithForward3 = false;
//        boolean doneWithForward4 = false;
//        boolean downWithTurn = false;
//        boolean doneWithMecanum2 = false;

        int idNumber = 1;

        double targetDistanceFromAprilTag = 10;

//        double[] powerMoveCloser = robot.calculateDrivetrainPower(25.4 * (targetDistanceFromAprilTag - robot.getAprilTagRange(idNumber)));
//        double[] powerMoveAway = robot.calculateDrivetrainPower(-25.4 * (targetDistanceFromAprilTag - robot.getAprilTagRange(idNumber)));


//        double[] stopPower = {0, 0, 0, 0};
        boolean isDone = false;


        while (opModeIsActive() && !isDone) {


            if (!robot.moveRelativeToAprilTagX(0, idNumber)) {
                isDone = robot.moveRelativeToAprilTagX(0, idNumber);
            }

            telemetry.addData("x is done", isDone);
            //robot.moveRelativeToAprilTag(10);

            telemetry.update();
        }
        isDone = false;
        while (opModeIsActive() && !isDone) {


            if (!robot.moveRelativeToAprilTagRange(10, idNumber)) {
                isDone = robot.moveRelativeToAprilTagRange(10, idNumber);
            }

            telemetry.addData("range", isDone);
            //robot.moveRelativeToAprilTag(10);

            telemetry.update();
        }

    }
}
