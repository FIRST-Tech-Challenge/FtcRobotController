package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class DriveTrainAutoExt extends LinearOpMode
{
    DriveTrain bot = new DriveTrain();

    @Override
    public void runOpMode()
    {
        bot.init();

    }
}
