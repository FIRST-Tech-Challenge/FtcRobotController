package org.firstinspires.ftc.team6220_2021.TestClasses;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team6220_2021.MasterOpMode;

@Disabled
@TeleOp(name = "TickTest", group = "TeleOp")
public class TickTest extends MasterOpMode {

    @Override
    public void runOpMode() {
        Initialize();

        servoGrabber.setPosition(0.34);
        pauseMillis(500);
        servoArm.setPosition(0.01);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                forward(72, 0.5);
            }
            else if (gamepad1.dpad_down) {
                turnAngle(90);
            }
        }
    }
}