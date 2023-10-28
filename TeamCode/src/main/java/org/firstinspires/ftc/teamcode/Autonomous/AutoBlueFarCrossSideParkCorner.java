package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutonomousOpenCV;

@Autonomous(name="Auto - Blue - Far - Cross side - Park corner", group="Auto - Blue")
public class AutoBlueFarCrossSideParkCorner extends AutonomousOpenCV {
    @Override
    public void runOpMode()
    {
        isRed = false;
        isNear = false;
        centerCross = false;
        cornerPark = true;
        super.runOpMode();
    }
}
