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
        double forwardPower = robot.calculateDrivetrainPower(150);

        while (opModeIsActive()) {
            armPower = robot.calculateArmPower(90);
            forwardPower = robot.calculateDrivetrainPower(150);


            if (!robot.checkReachedDistance(150)) {
                robot.setMotorPower(forwardPower, forwardPower, forwardPower, forwardPower);
            } else {
                robot.setMotorPower(0, 0, 0, 0);
            }

            if (!robot.checkArmPos(90)) {
                robot.setArmPower(armPower);
            } else {
                robot.setArmPower(0);
            }

        }

    }
}
