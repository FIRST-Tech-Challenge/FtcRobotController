package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Test Ball Detection - Red", group="Linear Opmode")
public class TestBallDetection_Red extends AutonomousOpenCV {
    @Override
    public void runOpMode()
    {
        detectionRed = true;
        runBallDetectionTest = true;
        super.runOpMode();
    }
}
