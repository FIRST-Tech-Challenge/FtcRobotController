package org.firstinspires.ftc.team8923_PowerPlay;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp Competition")

public class TeleOpCompetition extends BaseTeleOp{

    @Override
    public void runOpMode() throws InterruptedException{
        initHardware();
        waitForStart();

        while (opModeIsActive()){
            imuPivot();
            driveRobot();
            driveRobotSpeed();
            driveMechanism();
            driveClaw();
            idle();
        }
    }
}
