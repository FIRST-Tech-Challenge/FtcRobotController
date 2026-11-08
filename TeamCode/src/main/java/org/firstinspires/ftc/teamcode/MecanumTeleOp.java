package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Robot-centric mecanum drive.
 *   Left stick  Y : forward / backward
 *   Left stick  X : strafe left / right
 *   Right stick X : turn
 *
 * Motor names in the Robot Configuration must be: fl, fr, bl, br
 */
@TeleOp(name = "Mecanum TeleOp", group = "Drive")
public class MecanumTeleOp extends LinearOpMode {

    private DcMotor fl, fr, bl, br;

    @Override
    public void runOpMode() {
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");

        // Left side is usually reversed. If a wheel spins the wrong way when
        // pushing forward, flip that motor's direction here.
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotor m : new DcMotor[]{fl, fr, bl, br}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double y  = -gamepad1.left_stick_y;        // stick Y is reversed
            double x  =  gamepad1.left_stick_x * 1.1;  // counter imperfect strafing
            double rx =  gamepad1.right_stick_x;

            // Normalize so no power exceeds 1 while keeping the ratio
            double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

            double flPower = (y + x + rx) / denom;
            double blPower = (y - x + rx) / denom;
            double frPower = (y - x - rx) / denom;
            double brPower = (y + x - rx) / denom;

            fl.setPower(flPower);
            bl.setPower(blPower);
            fr.setPower(frPower);
            br.setPower(brPower);

            telemetry.addData("fl / fr", "%.2f / %.2f", flPower, frPower);
            telemetry.addData("bl / br", "%.2f / %.2f", blPower, brPower);
            telemetry.update();
        }
    }
}
