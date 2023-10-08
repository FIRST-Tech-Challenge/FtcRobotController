package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Test Encoders", group="Linear Opmode")
public class TestEncoders extends AutonomousOpenCV {
    @Override
    public void runOpMode()
    {
        runEncoderTest = true;
        super.runOpMode();
    }
}
