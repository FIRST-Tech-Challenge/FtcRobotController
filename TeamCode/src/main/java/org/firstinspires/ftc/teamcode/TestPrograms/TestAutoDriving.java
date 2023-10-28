package org.firstinspires.ftc.teamcode.TestPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutonomousOpenCV;

@Autonomous(name="Test Auto Driving", group="Auto Test")
public class TestAutoDriving extends AutonomousOpenCV {
    @Override
    public void runOpMode()
    {
        runAutoDrivingTest = true;
        super.runOpMode();
    }
}
