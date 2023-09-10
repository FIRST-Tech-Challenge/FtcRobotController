package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Ethan.EthanRobot;

@Autonomous
public class AutoRed extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, this, telemetry);
        waitForStart();

        while (opModeIsActive()) {
            robot.autoMecanuming(150);
            if  (robot.checkReachedDistance(150, true)) {
                break;
            }
        }

    }
}
