package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Test Auto Driving", group="Linear Opmode")
public class TestAutoDriving extends AutonomousOpenCV {
    @Override
    public void runOpMode()
    {
        runAutoDrivingTest = true;
        super.runOpMode();
    }
}
