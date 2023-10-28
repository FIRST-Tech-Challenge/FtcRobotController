package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutonomousOpenCV;

@Autonomous(name="Auto - Blue - Near - Park Middle", group="Auto - Blue")
public class AutoBlueNearParkMiddle extends AutonomousOpenCV {
    @Override
    public void runOpMode()
    {
        isRed = false;
        isNear = true;
        cornerPark = false;
        super.runOpMode();
    }

}
