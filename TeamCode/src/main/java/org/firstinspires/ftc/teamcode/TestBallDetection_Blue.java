package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Test Ball Detection - Blue", group="Linear Opmode")
public class TestBallDetection_Blue extends AutonomousOpenCV {
    @Override
    public void runOpMode()
    {
        isRed = false;
        runBallDetectionTest = true;
        super.runOpMode();
    }
}
