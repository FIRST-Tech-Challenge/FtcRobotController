package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Test Ball Detection - Blue", group="Linear Opmode")
public class TestBallDetection_Blue extends AutonomousOpenCV {
    @Override
    public void runOpMode()
    {
        detectionRed = false;
        runBallDetectionTest = true;
        super.runOpMode();
    }
}
