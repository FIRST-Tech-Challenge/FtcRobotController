package org.firstinspires.ftc.teamcode.TestPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutonomousOpenCV;

@Autonomous(name="Test Ball Detection - Red", group="Auto Test")
public class TestBallDetection_Red extends AutonomousOpenCV {
    @Override
    public void runOpMode()
    {
        isRed = true;
        runBallDetectionTest = true;
        super.runOpMode();
    }
}
