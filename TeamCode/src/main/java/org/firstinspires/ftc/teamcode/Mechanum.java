package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Mechanum extends LinearOpMode {
    private
    private Gamepad gamepad1 = new Gamepad();
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
            double power = Math.sqrt(y^2+rx^2);
            double x = Math.atan2(y, rx);

            if (gamepad.right_stick_x != ){
                frontRight.setPower(x - (1/4) * Math.PI);
                backLeft.setPower(x - (1/4) * Math.PI);

                frontLeft.setPower(x + (1/4) * Math.PI);
                backRight.setPower(x + (1/4) * Math.PI);
            } else {
                frontLeft.setPower(x);
                backLeft.setPower(x);

                frontRight.setPower(-x);
                backRight.setPower(-x);
            }
        }
    }


}