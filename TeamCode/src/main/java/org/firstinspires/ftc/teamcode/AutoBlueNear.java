package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Auto - Blue - Near", group="Linear Opmode")
public class AutoBlueNear extends AutonomousOpenCV {
    @Override
    public void runOpMode()
    {
        isRed = false;
        isNear = true;
        super.runOpMode();
    }

}
