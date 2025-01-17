package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;

public class WheelControlTest extends LinearOpMode {

    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait to press START on controller
        waitForStart();

        while (opModeIsActive()) {
            double JOYSTICK_MIN;
            double JOYSTICK_MAX;

            double y  = -gamepad1.left_stick_y;
            double rx = gamepad1.left_stick_x;
            double power = Math.sqrt(y*y+rx*rx);
            double x = Math.atan2(y, rx);
            frontRight.setPower(x - (0.25f) * Math.PI);
            backLeft.setPower(x - (0.25f) * Math.PI);

            frontLeft.setPower(x + (0.25f) * Math.PI);
            backRight.setPower(x + (0.25f) * Math.PI);
        }
    }


}

