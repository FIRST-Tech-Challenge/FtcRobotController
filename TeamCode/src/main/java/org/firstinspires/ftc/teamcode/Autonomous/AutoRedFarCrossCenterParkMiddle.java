package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutonomousOpenCV;

@Autonomous(name="Auto - Red - Far - Cross Center - Park Middle", group="Auto - Red")
public class AutoRedFarCrossCenterParkMiddle extends AutonomousOpenCV {
    @Override
    public void runOpMode()
    {
        isRed = true;
        isNear = false;
        centerCross = true;
        cornerPark = false;
        useAprilTagsToDeliverPixel = true;
        super.runOpMode();
    }
}

