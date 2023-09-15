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



        while (opModeIsActive()) {
            double armPower = robot.calculateArmPower(30);
            robot.setArmPower(armPower);

            if (robot.checkArmPos(30)) {
                break;
            }

        }

    }
}
