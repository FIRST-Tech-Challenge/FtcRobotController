package org.firstinspires.ftc.team8923_PowerPlay;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp Competition")

abstract public class TeleOpCompetition extends BaseTeleOp{

    public void runOpmode() throws InterruptedException{
        initHardware();
        waitForStart();

        while (opModeIsActive()){
            driveRobot();
            driveRobotSpeed();
            idle();
        }
    }
}
