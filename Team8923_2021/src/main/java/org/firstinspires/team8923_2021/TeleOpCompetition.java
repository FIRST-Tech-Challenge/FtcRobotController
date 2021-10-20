package org.firstinspires.team8923_2021;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp Competition")

public class TeleOpCompetition extends MasterTeleOp
{
    @Override
    public void runOpMode()
    {
        initHardware();
        waitForStart();

        while (opModeIsActive())
        {
            splitArcadeDrive();
            runDriveSpeed();
            idle();
        }
    }
}
