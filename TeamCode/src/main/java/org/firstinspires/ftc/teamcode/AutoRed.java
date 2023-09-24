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
        waitForStart();

//        double armPower = robot.calculateArmPower(90);
        double[] forwardPower = robot.calculateDrivetrainPowerWithTurning(600, 30);
        double[] mecanumPower = robot.calculateMecanumPower(0);
        double[] backwardPower = robot.calculateDrivetrainPower(0);

        double mecanumDistance = -711.2;
        double forwardDistance = 600;

        boolean doneWithMecanum = false;
        boolean doneWithForward = false;
        boolean doneWithBackward = false;


        while (opModeIsActive()) {

            telemetry.addLine("power       " + forwardPower[0]);
            telemetry.addLine("power       " + forwardPower[1]);
            telemetry.addLine("power       " + forwardPower[2]);
            telemetry.addLine("power       " + forwardPower[3]);

            /*if (!doneWithMecanum && !robot.checkReachedDistanceForMecanum(mecanumDistance)) {
                mecanumPower = robot.calculateMecanumPower(mecanumDistance);

                robot.setMotorPower(mecanumPower);
            } else if (!doneWithMecanum){
                robot.setMotorPower(0, 0, 0, 0);
                robot.resetEncoder();
                doneWithMecanum = true;
            }
            telemetry.addData("is done wit macanum", doneWithMecanum);*/

            doneWithMecanum = true;
            if (!doneWithForward && doneWithMecanum && !robot.checkReachedDistance(forwardDistance)) {
                forwardPower = robot.calculateDrivetrainPowerWithTurning(forwardDistance, 30);
                robot.setMotorPower(forwardPower);
            } else if (!doneWithForward && doneWithMecanum){
                robot.setMotorPower(0, 0, 0, 0);
                robot.resetEncoder();
                doneWithForward = true;
            }
            telemetry.addData("is done with forward", doneWithForward);

            /*if (!doneWithBackward && doneWithMecanum && doneWithForward && !robot.checkReachedDistance(-forwardDistance)) {
                backwardPower = robot.calculateDrivetrainPower(-forwardDistance);
                robot.setMotorPower(backwardPower);
            } else if (!doneWithBackward && doneWithForward) {
                robot.setMotorPower(0, 0, 0, 0);
                robot.resetEncoder();
                doneWithBackward = true;
            }

            telemetry.addData("done with backward", doneWithBackward);*/

            /*if (!robot.checkArmPos(90)) {
                armPower = robot.calculateArmPower(90);
                robot.setArmPower(armPower);
            } else if (robot.checkArmPos(90)){
                robot.setArmPower(0);
            }
*/
            telemetry.update();
        }
    }
}
