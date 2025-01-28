package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "parkTwoFeetAutoProgram")
public class parkTwoFeetAutoProgram extends LinearOpMode {
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private static ElapsedTime myStopwatch = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Initialize the hardware
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        // Wait for the start signal
        waitForStart();

        frontLeft.setDirection(DcMotorEx.Direction.FORWARD);
        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.FORWARD);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);

        myStopwatch.reset();
        // robot travels 20 inches in 1 sec
        // goes 47.5 inches in 2 sec
        // goes 66 inches in 3 sec
        while (myStopwatch.seconds() < 1.0) {
            frontLeft.setPower(1.0);
            frontRight.setPower(-1.0);
            backLeft.setPower(-0.55);
            backRight.setPower(0.45);
            telemetry.addData("Status", "Strafing Right");
            telemetry.update();
        }

        stopMotors();
    }

    private void stopMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}