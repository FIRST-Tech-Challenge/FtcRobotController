package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutonomousOpenCV;

@Autonomous(name="Auto - Red - Far - Cross Side - Park Corner", group="Auto - Red")
public class AutoRedFarCrossSideParkCorner extends AutonomousOpenCV {
    @Override
    public void runOpMode()
    {
        isRed = true;
        isNear = false;
        cornerPark = true;
        centerCross = false;
        super.runOpMode();
    }
}
