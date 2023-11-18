package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Disabled

@Autonomous(name = "AutoMotor")
public class RFMotorAutoTuneTest extends AutoRFMotorTest{
    @Override
    public void runOpMode() throws InterruptedException {
        initialize("leftLiftMotor", 1000, 0, DcMotorSimple.Direction.REVERSE);
        waitForStart();
        auto();
    }
}
