package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Drive Forward Test", group = "Test")
public class DriveForwardTest extends LinearOpMode {

    private static final double MAX_POWER = 0.4;

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
            double targetPower = -gamepad1.left_stick_y * MAX_POWER;

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
