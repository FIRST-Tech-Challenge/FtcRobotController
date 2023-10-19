package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Auto - Red - Far", group="Linear Opmode")
public class AutoRedFar extends AutonomousOpenCV {
    @Override
    public void runOpMode()
    {
        isRed = true;
        isNear = false;
        super.runOpMode();
    }
}
