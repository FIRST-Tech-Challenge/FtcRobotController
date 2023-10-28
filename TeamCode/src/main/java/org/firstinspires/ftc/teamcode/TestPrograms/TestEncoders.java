package org.firstinspires.ftc.teamcode.TestPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonomousOpenCV;

@Autonomous(name="Test Encoders", group="Auto Test")
public class TestEncoders extends AutonomousOpenCV {
    @Override
    public void runOpMode()
    {
        runEncoderTest = true;
        super.runOpMode();
    }
}
