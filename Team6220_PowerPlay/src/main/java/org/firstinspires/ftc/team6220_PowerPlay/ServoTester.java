package org.firstinspires.ftc.team6220_PowerPlay;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "servo")
public class ServoTester extends BaseTeleOp{
    @Override
    public void runOpMode() throws InterruptedException {

        if (gamepad2.dpad_up) {
            servoPos += 0.5;
        } else if (gamepad2.dpad_down) {
            servoPos -= 0.5;
        }
        servoGrabber.setPosition(servoPos);
        telemetry.addData("servoPos", servoPos);
        telemetry.update();
    }
}
