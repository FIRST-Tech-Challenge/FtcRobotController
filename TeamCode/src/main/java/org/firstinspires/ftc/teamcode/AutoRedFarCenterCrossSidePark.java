package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Auto - Red - Far - CenterCross- SidePark", group="Linear Opmode")
public class AutoRedFarCenterCrossSidePark extends AutonomousOpenCV {
    @Override
    public void runOpMode()
    {
        isRed = true;
        isNear = false;
        //centerCross = true;
        cornerPark = false;
        super.runOpMode();
    }
}

