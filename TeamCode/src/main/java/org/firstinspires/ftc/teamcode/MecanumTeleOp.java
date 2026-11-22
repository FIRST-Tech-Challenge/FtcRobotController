package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * Mecanum drivetrain TeleOp — REV Control Hub
 *
 * Controls (gamepad1):
 *   Left stick        – drive (forward/back + strafe)
 *   Right stick X     – turn
 *   Right bumper      – slow mode (hold)
 *   A                 – toggle field-centric / robot-centric
 *   Options / Start   – reset IMU heading (field-centric "forward")
 *
 * Robot configuration names (set these on the Driver Station):
 *   frontLeft, backLeft, frontRight, backRight, imu
 */
@TeleOp(name = "Mecanum TeleOp", group = "Drive")
public class MecanumTeleOp extends LinearOpMode {

    // Tuning
    private static final double SLOW_MODE_SCALE = 0.4;   // speed when right bumper held
    private static final double SUPER_SLOW_MODE_SCALE = 0.2; //for alex R
    private static final double STRAFE_CORRECTION = 1.1; // counteracts mecanum strafe losses
    private static final double DEADZONE = 0.05;

    private DcMotor frontLeft, backLeft, frontRight, backRight;
    private IMU imu;

    @Override
    public void runOpMode() {
        // ---------- Hardware ----------
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        // Most setups need ONE side reversed. If the robot spins when you push
        // forward, or a wheel fights the others, flip that motor's direction here.
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotor m : new DcMotor[]{frontLeft, backLeft, frontRight, backRight}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Built-in Control Hub IMU. Change these two values to match how the
        // hub is physically mounted on the robot.
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw();

        boolean fieldCentric = false;
        boolean aWasPressed = false;

        telemetry.addLine("Ready. Press START.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // ---------- Main loop ----------
        while (opModeIsActive()) {

            // Toggle drive mode on A (edge-detected so it doesn't flicker)
            if (gamepad1.a && !aWasPressed) fieldCentric = !fieldCentric;
            aWasPressed = gamepad1.a;

            // Reset heading — point robot "away from you" and press this
            if (gamepad1.options) imu.resetYaw();

            // Sticks: Y is inverted on gamepads (up = negative)
            double y  = deadzone(-gamepad1.left_stick_y);
            double x  = deadzone(gamepad1.left_stick_x);
            double rx = deadzone(-gamepad1.right_stick_x);

            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            if (fieldCentric) {
                // Rotate the joystick vector by -heading
                double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
                double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);
                x = rotX;
                y = rotY;
            }

            x *= STRAFE_CORRECTION;

            double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
            double fl = (y + x + rx) / denom;
            double bl = (y - x + rx) / denom;
            double fr = (y - x - rx) / denom;
            double br = (y + x - rx) / denom;

            double scale = gamepad1.right_bumper ? SLOW_MODE_SCALE :
                    gamepad1.left_bumper ? SUPER_SLOW_MODE_SCALE : 1.0;

            frontLeft.setPower(fl * scale);
            backLeft.setPower(bl * scale);
            frontRight.setPower(fr * scale);
            backRight.setPower(br * scale);

            // ---------- Telemetry ----------
            telemetry.addData("Mode", fieldCentric ? "FIELD-CENTRIC" : "ROBOT-CENTRIC");
            telemetry.addData("Slow mode", gamepad1.right_bumper);
            telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(heading));
            telemetry.addData("FL / FR", "%.2f / %.2f", fl * scale, fr * scale);
            telemetry.addData("BL / BR", "%.2f / %.2f", bl * scale, br * scale);
            telemetry.update();
        }
    }

    private double deadzone(double v) {
        return Math.abs(v) < DEADZONE ? 0.0 : v;
    }
}