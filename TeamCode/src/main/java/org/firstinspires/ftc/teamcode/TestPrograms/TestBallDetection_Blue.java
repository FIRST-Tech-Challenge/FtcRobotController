package org.firstinspires.ftc.teamcode.TestPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutonomousOpenCV;

@Autonomous(name="Test Ball Detection - Blue", group="Auto Test")
public class TestBallDetection_Blue extends AutonomousOpenCV {
    @Override
    public void runOpMode()
    {
        isRed = false;
        runBallDetectionTest = true;
        super.runOpMode();
    }
}
