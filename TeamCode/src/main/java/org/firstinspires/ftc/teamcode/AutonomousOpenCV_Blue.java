package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Autonomous - Blue ball", group="Linear Opmode")
public class AutonomousOpenCV_Blue extends AutonomousOpenCV {
    @Override
    public void runOpMode()
    {
        detectionRed = false;
        super.runOpMode();
    }
}
