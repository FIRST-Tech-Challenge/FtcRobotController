package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutonomousOpenCV;

@Autonomous(name="Auto - Red - Near - Park Corner", group="Auto - Red")
public class AutoRedNearParkCorner extends AutonomousOpenCV {
    @Override
    public void runOpMode()
    {
        isRed = true;
        isNear = true;
        cornerPark = true;
        super.runOpMode();
    }
}
