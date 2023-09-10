package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class DriveMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // Motor config
        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeft = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRight = hardwareMap.dcMotor.get("backRight");

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // For forwards/backwards movement
            double x = gamepad1.left_stick_x * 1.1; // The 1.1 multiplier is to counteract imperfect strafing
            double rx = gamepad1.right_stick_x; // Turning left/right

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1); /* makes sure motor values don't go outside of [-1,1] */
            double fL_Motor = (y+x+rx)/denominator; // fL = FrontLeft
            double bL_Motor = (y-x+rx)/denominator; // bL = BackLeft
            double fR_Motor = (y-x-rx)/denominator; // fR = FrontRight
            double bR_Motor = (y+x-rx)/denominator; // bR = backRight

            frontLeft.setPower(fL_Motor);
            backLeft.setPower(bL_Motor);
            frontRight.setPower(fR_Motor);
            backRight.setPower(bR_Motor);

        }
    }
}
