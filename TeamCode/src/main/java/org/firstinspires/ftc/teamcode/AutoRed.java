package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class AutoRed extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, this, telemetry);
        robot.setUpDrivetrainMotors();
//        robot.setUpArmMotor();

        robot.setUpAprilTags();
        waitForStart();


        double mecanumDistance = -711.2;
        double forwardDistance = 711.2;
        double forwardDistance2 = -482.6;
        double forwardDistance3 = 889;
        double forwardDistance4 = 2438.4;
        double turnDegrees1 = 90;
        double mecanumDistance2 = 609.6;



//        double armPower = robot.calculateArmPower(90);
        double[] forwardPower = robot.calculateDrivetrainPower(forwardDistance);
        double[] mecanumPower = robot.calculateMecanumPower(mecanumDistance);
        double[] forwardPower2 = robot.calculateDrivetrainPower(forwardDistance2);
        double[] forwardPower3 = robot.calculateDrivetrainPower(forwardDistance3);
        double[] forwardPower4 = robot.calculateDrivetrainPower(forwardDistance4);
        double[] mecanumPower2 = robot.calculateMecanumPower(mecanumDistance2);


        boolean doneWithMecanum = false;
        boolean doneWithForward = false;
        boolean doneWithForward2 = false;
        boolean doneWithForward3 = false;
        boolean doneWithForward4 = false;
        boolean downWithTurn = false;
        boolean doneWithMecanum2 = false;

        double targetDistanceFromAprilTag = 10;

        double[] powerMoveCloser = robot.calculateDrivetrainPower(25.4*(targetDistanceFromAprilTag - robot.getAprilTagRange()));
        double[] powerMoveAway = robot.calculateDrivetrainPower(-25.4*(targetDistanceFromAprilTag - robot.getAprilTagRange()));


        double[] stopPower = {0, 0, 0, 0};
            /*
            //Forward 1
            if (!doneWithForward && !robot.checkReachedDistance(forwardDistance)) {
                forwardPower = robot.calculateDrivetrainPower(forwardDistance);
                robot.setMotorPower(forwardPower);
            } else if (!doneWithForward){
                robot.setMotorPower(0, 0, 0, 0);
                robot.resetEncoder();
                doneWithForward = true;
            }

            telemetry.addData("is done with forward1", doneWithForward);

            //Forward 2
            if (!doneWithForward2 && doneWithForward && !robot.checkReachedDistance(forwardDistance2)) {
                forwardPower2 = robot.calculateDrivetrainPower(forwardDistance2);
                robot.setMotorPower(forwardPower2);
            } else if (!doneWithForward2 && doneWithForward) {
                robot.setMotorPower(0, 0, 0, 0);
                robot.resetEncoder();
                doneWithForward2 = true;
            }

            telemetry.addData("done with forward 2", doneWithForward2);

            //Mecanum 1
            if (!doneWithMecanum && doneWithForward2 && !robot.checkReachedDistanceForMecanum(mecanumDistance)) {
                mecanumPower = robot.calculateMecanumPower(mecanumDistance);
                robot.setMotorPower(mecanumPower);
            } else if (!doneWithMecanum && doneWithForward2){
                robot.setMotorPower(0, 0, 0, 0);
                robot.resetEncoder();
                doneWithMecanum = true;
            }

            telemetry.addData("is done wit macanum", doneWithMecanum);

            //Forward 3
            if (!doneWithForward3 && doneWithMecanum && !robot.checkReachedDistance(forwardDistance3)) {
                forwardPower3 = robot.calculateDrivetrainPower(forwardDistance3);
                robot.setMotorPower(forwardPower3);
            } else if (doneWithMecanum) {
                robot.setMotorPower(0, 0, 0, 0);
                robot.resetEncoder();
                doneWithForward3 = true;
            }

            telemetry.addData("is done with forward 3", doneWithForward3);

            //Turn 1
            if (doneWithForward3 && !downWithTurn) {
                robot.setHeading(turnDegrees1);
                if (robot.isHeadingWithinError(turnDegrees1)) {
                    downWithTurn = true;
                }
            }

            //Forward 4
            if (!doneWithForward4 && doneWithMecanum && !robot.checkReachedDistance(forwardDistance4)) {
                forwardPower = robot.calculateDrivetrainPower(forwardDistance4);
                robot.setMotorPower(forwardPower4);
            } else if (!doneWithForward4 && doneWithMecanum){
                robot.setMotorPower(0, 0, 0, 0);
                robot.resetEncoder();
                doneWithForward4 = true;
            }

            telemetry.addData("is done with forward 4", doneWithForward4);


            if (!doneWithMecanum2 && doneWithForward4 && !robot.checkReachedDistanceForMecanum(mecanumDistance2)) {
                mecanumPower2 = robot.calculateMecanumPower(mecanumDistance2);
                robot.setMotorPower(mecanumPower2);
            } else if (!doneWithMecanum2 && doneWithForward4){
                robot.setMotorPower(0, 0, 0, 0);
                robot.resetEncoder();
                doneWithMecanum2 = true;
            }

            telemetry.addData("is done wit macanum 2", doneWithMecanum2);
*/


        while (opModeIsActive()) {

            if (
                !(robot.getAprilTagRange() > targetDistanceFromAprilTag - 0.5 &&
                robot.getAprilTagRange() < targetDistanceFromAprilTag + 0.5) &&
                robot.getAprilTagRange() < targetDistanceFromAprilTag
            ){

                powerMoveAway = robot.calculateDrivetrainPower(25.4*(targetDistanceFromAprilTag - robot.getAprilTagRange()));
                robot.setMotorPower(powerMoveAway);

            } else if (
                       !(robot.getAprilTagRange() > targetDistanceFromAprilTag - 0.5 &&
                       robot.getAprilTagRange() < targetDistanceFromAprilTag + 0.5) &&
                       robot.getAprilTagRange() > targetDistanceFromAprilTag
            ){

                powerMoveCloser = robot.calculateDrivetrainPower(-25.4*(targetDistanceFromAprilTag - robot.getAprilTagRange()));
                robot.setMotorPower(powerMoveCloser);
            } else if (
                       robot.getAprilTagRange() > targetDistanceFromAprilTag - 0.5 &&
                       robot.getAprilTagRange() < targetDistanceFromAprilTag + 0.5
            ){
                robot.setMotorPower(stopPower);
            }


            //robot.moveRelativeToAprilTag(10);

            telemetry.update();
        }
    }
}
