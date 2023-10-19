package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Auto - Red - Near", group="Linear Opmode")
public class AutoRedNear extends AutonomousOpenCV {
    @Override
    public void runOpMode()
    {
        isRed = true;
        isNear = true;
        super.runOpMode();
    }
}
