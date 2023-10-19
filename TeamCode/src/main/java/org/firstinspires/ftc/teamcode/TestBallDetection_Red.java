package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Test Ball Detection - Red", group="Linear Opmode")
public class TestBallDetection_Red extends AutonomousOpenCV {
    @Override
    public void runOpMode()
    {
        isRed = true;
        runBallDetectionTest = true;
        super.runOpMode();
    }
}
