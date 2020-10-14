package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "forwardTwelveInches", group = "Robot")
public class forwardTwelveInches extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(telemetry, hardwareMap, this);
        waitForStart();
        robot.forwardTwelveInches();

    }
}
