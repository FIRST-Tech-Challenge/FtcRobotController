package org.firstinspires.ftc.team6220_2021.TestClasses;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team6220_2021.MasterOpMode;

@Disabled
@TeleOp(name = "TeleOpTest", group = "Test")
public class TeleOpTest extends MasterOpMode {

    @Override
    public void runOpMode() {
        Initialize();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                telemetry.addData("servo grabber", servoGrabber.getPosition());
                telemetry.addData("servo arm", servoArm.getPosition());
                telemetry.addData("motor arm", motorArm.getCurrentPosition());
                telemetry.update();
            }
        }
    }
}