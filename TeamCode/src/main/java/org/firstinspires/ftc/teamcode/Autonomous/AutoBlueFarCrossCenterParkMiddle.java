package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutonomousOpenCV;

@Autonomous(name="Auto - Blue - Far - Cross Center - Park Middle", group="Auto - Blue")
public class AutoBlueFarCrossCenterParkMiddle extends AutonomousOpenCV {
    @Override
    public void runOpMode()
    {
        isRed = false;
        isNear = false;
        centerCross = true;
        cornerPark = false;
        useAprilTagsToDeliverPixel = true;
        super.runOpMode();
    }
}
