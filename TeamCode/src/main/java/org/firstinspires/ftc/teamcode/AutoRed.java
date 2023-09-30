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


        double mecanumDistance = -711.2;
        double forwardDistance = 150;

        double forwardDistanceNumeroDuos = 450;


//        double armPower = robot.calculateArmPower(90);
        double[] forwardPower = robot.calculateDrivetrainPower(forwardDistance);
        double[] mecanumPower = robot.calculateMecanumPower(0);
        double[] forwardPowerNumeroDuos = robot.calculateDrivetrainPower(forwardDistanceNumeroDuos);

        boolean doneWithMecanum = false;
        boolean doneWithForward = false;
        boolean doneWithForward2 = false;



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
                forwardPower = robot.calculateDrivetrainPower(forwardDistance);
                robot.setMotorPower(forwardPower);
            } else if (!doneWithForward && doneWithMecanum){
                robot.setMotorPower(0, 0, 0, 0);
                robot.resetEncoder();
                doneWithForward = true;
            }

            telemetry.addData("is done with forward", doneWithForward);

            if (doneWithForward) {
                robot.setHeading(30);
            }

            //removed boolean setheading

            /*if (!robot.checkArmPos(90)) {
                armPower = robot.calculateArmPower(90);
                robot.setArmPower(armPower);
            } else if (robot.checkArmPos(90)){
                robot.setArmPower(0);
            }
*/
            telemetry.update();
        }

        while (opModeIsActive()) {

            if (!doneWithForward2 && doneWithMecanum && doneWithForward && !robot.checkReachedDistance(forwardDistanceNumeroDuos)) {
                forwardPowerNumeroDuos = robot.calculateDrivetrainPower(forwardDistanceNumeroDuos);
                robot.setMotorPower(forwardPowerNumeroDuos);
            } else if (!doneWithForward2 && doneWithForward) {
                robot.setMotorPower(0, 0, 0, 0);
                robot.resetEncoder();
                doneWithForward2 = true;
            }

            telemetry.addData("done with backward", doneWithForward2);



            /*if (!robot.checkArmPos(90)) {
                armPower = robot.calculateArmPower(90);
                robot.setArmPower(armPower);
            } else if (robot.checkArmPos(90)){
                robot.setArmPower(0);
            }
*/
            telemetry.update();
        }

        while (opModeIsActive()) {


        }

    }
}
