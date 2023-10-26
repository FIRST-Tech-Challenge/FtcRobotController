package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Auto - Blue - Far - CenterCross- SidePark", group="Linear Opmode")
public class AutoBlueFarCenterSidePark extends AutonomousOpenCV {
    @Override
    public void runOpMode()
    {
        isRed = false;
        isNear = false;
        //centerCross = true;
        cornerPark = false;
        super.runOpMode();
    }
}
