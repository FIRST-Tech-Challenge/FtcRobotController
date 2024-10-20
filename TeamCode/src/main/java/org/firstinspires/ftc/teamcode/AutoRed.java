package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Ethan.EthanRobot;

@Autonomous
public class AutoRed extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, this, telemetry);
        robot.setUpDrivetrainMotors();
        robot.setUpArmMotor();
        waitForStart();

        double armPower = robot.calculateArmPower(90);
        double[] forwardBackwardPower = robot.calculateDrivetrainPower(300);

        while (opModeIsActive()) {
            armPower = robot.calculateArmPower(90);
            forwardBackwardPower = robot.calculateDrivetrainPower(300);

            telemetry.addLine("power       " + forwardBackwardPower[0]);
            telemetry.addLine("power       " + forwardBackwardPower[1]);
            telemetry.addLine("power       " + forwardBackwardPower[2]);
            telemetry.addLine("power       " + forwardBackwardPower[3]);


            if (!robot.checkReachedDistance(600)) {
                robot.setMotorPower(forwardBackwardPower[0], forwardBackwardPower[1], forwardBackwardPower[2], forwardBackwardPower[3]);
            } else {
                robot.setMotorPower(0, 0, 0, 0);
            }

            if (!robot.checkArmPos(90)) {
                robot.setArmPower(armPower);
            } else {
                robot.setArmPower(0);
            }

            telemetry.update();
        }

    }
}
