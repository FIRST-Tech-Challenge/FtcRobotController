package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Auto - Blue - Far", group="Linear Opmode")
public class AutoBlueFar extends AutonomousOpenCV {
    @Override
    public void runOpMode()
    {
        isRed = false;
        isNear = false;
        super.runOpMode();
    }
}
