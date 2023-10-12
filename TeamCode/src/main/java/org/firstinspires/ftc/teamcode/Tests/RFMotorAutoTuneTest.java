package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Autonomous(name = "AutoMotor")
public class RFMotorAutoTuneTest extends AutoRFMotorTest{


    @Override
    public void runOpMode() throws InterruptedException {
        initialize("leftLiftMotor", 3000, 0, DcMotorSimple.Direction.REVERSE);
        waitForStart();
        auto();
    }
}
