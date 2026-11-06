package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Drive Forward Test", group = "Test")
public class DriveForwardTest extends LinearOpMode {

    // Highest power the wheels are allowed to reach (0.0 - 1.0). Keep it low
    // so the test stays gentle on the drivetrain.
    private static final double MAX_POWER = 0.4;

    // Most the power is allowed to change per loop. Smaller = smoother/slower
    // ramp, so the chain never gets a sudden jerk.
    private static final double RAMP_RATE = 0.02;

    @Override
    public void runOpMode() {
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("Ready. Press PLAY, then push the left stick.");
        telemetry.update();

        waitForStart();

        double currentPower = 0.0;
        while (opModeIsActive()) {
            // Where the stick wants us to be (up = forward), scaled to MAX_POWER.
            double targetPower = -gamepad1.left_stick_y * MAX_POWER;

            // Step currentPower toward targetPower, but never by more than
            // RAMP_RATE in one loop -- this is what makes it gradual.
            double diff = targetPower - currentPower;
            if (diff > RAMP_RATE) {
                currentPower += RAMP_RATE;
            } else if (diff < -RAMP_RATE) {
                currentPower -= RAMP_RATE;
            } else {
                currentPower = targetPower;
            }

            backLeft.setPower(currentPower);
            backRight.setPower(currentPower);

            telemetry.addData("Stick target", "%.2f", targetPower);
            telemetry.addData("Actual power", "%.2f", currentPower);
            telemetry.update();

            sleep(20);
        }
    }
}
