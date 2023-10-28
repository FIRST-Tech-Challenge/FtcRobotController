package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutonomousOpenCV;

@Autonomous(name="Auto - Blue - Near - Park Corner", group="Auto - Blue")
public class AutoBlueNearParkCorner extends AutonomousOpenCV {
    @Override
    public void runOpMode()
    {
        isRed = false;
        isNear = true;
        cornerPark = true;
        super.runOpMode();
    }

}
