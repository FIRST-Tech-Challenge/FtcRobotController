package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutonomousOpenCV;

@Autonomous(name="Auto - Red - Near - Park Middle", group="Auto - Red")
public class AutoRedNearParkMiddle extends AutonomousOpenCV {
    @Override
    public void runOpMode()
    {
        isRed = true;
        isNear = true;
        cornerPark = false;
        super.runOpMode();
    }
}
