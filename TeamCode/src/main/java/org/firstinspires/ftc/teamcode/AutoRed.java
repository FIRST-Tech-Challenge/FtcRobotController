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
        double[] mecanumPower = robot.calculateMecanumPower(300);

        while (opModeIsActive()) {
            armPower = robot.calculateArmPower(90);
            mecanumPower = robot.calculateMecanumPower(600);

            telemetry.addLine("mecanum power       " + mecanumPower[0]);
            telemetry.addLine("mecanum power       " + mecanumPower[1]);
            telemetry.addLine("mecanum power       " + mecanumPower[2]);
            telemetry.addLine("mecanum power       " + mecanumPower[3]);


            if (!robot.checkReachedDistanceForMecanum(600)) {
                robot.setMotorPower(mecanumPower);
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
