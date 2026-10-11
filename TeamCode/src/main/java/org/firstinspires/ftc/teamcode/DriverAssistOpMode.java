package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Robot-Centric Driver Assist Mode with Macro Controls
 *
 * Controls:
 * - Left Stick: Move the robot (Robot-Centric style: Up drives straight forward)
 * - Right Stick: Manual turning (Only works when heading lock is turned off)
 * - Y / B / A / X: Lock facing direction to 0 / 90 / 180 / 270 degrees
 * - Left Bumper: Snap to the nearest 90-degree angle
 * - Right Bumper: Turn off the heading lock (Returns full control to right stick)
 * - Right Trigger: HOLD to automatically score (Drives straight toward target)
 */
@TeleOp(name = "LQR Driver Assist")
public class DriverAssistOpMode extends LinearOpMode {

    // Safety limitations and stick configurations parameters
    // 312 rpm numbers, same as PathFollowerOpMode (these were the 435 rpm numbers)
    private static final double MAX_V = 46.0;
    private static final double MAX_A = 40.0;
    private static final double MAX_ANG = 3.3;

    private static final double SCORE_X = 48.0;
    private static final double SCORE_Y = 24.0;
    private static final double SCORE_HEADING = Math.PI / 2.0;
    private static final double STICK_D = 0.05;

    // Drivetrain hardware references
    private DcMotor fL, fR, bL, bR;

    @Override
    public void runOpMode() throws InterruptedException {
        // Connect to physical hardware configuration names
        fL = hardwareMap.get(DcMotor.class, "fL");
        fR = hardwareMap.get(DcMotor.class, "fR");
        bL = hardwareMap.get(DcMotor.class, "bL");
        bR = hardwareMap.get(DcMotor.class, "bR");

        // Drivetrain direction mapping parameters
        fL.setDirection(DcMotor.Direction.REVERSE);
        bL.setDirection(DcMotor.Direction.REVERSE);

        // Without this the motors coast when power hits zero instead of resisting, which is the "spins a bit before stopping".
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Core calculation helper subsystems
        Localizer localizer = createLocalizer();
        DriverAssist assist = DriverAssist.withDefaults(MAX_ANG);
        LQRPathFollower macro = LQRPathFollower.withDefaults(MAX_V, MAX_A, MAX_ANG);

        boolean macroActive = false;
        ElapsedTime loopTimer = new ElapsedTime();

        waitForStart();
        loopTimer.reset();

        while (opModeIsActive()) {
            // Calculate active runtime clock delta intervals
            double dt = loopTimer.seconds();
            loopTimer.reset();
            if (dt <= 0) {
                dt = 0.02;
            }

            // Extract baseline coordinate mapping outputs
            localizer.update();
            Pose pose = localizer.getPose();

            // Check face buttons to assign orientation lock boundaries
            if (gamepad1.y) assist.lockHeading(0);
            if (gamepad1.b) assist.lockHeading(Math.PI / 2);
            if (gamepad1.a) assist.lockHeading(Math.PI);
            if (gamepad1.x) assist.lockHeading(-Math.PI / 2);

            // Left bumper handles snap loops, right bumper breaks the lock active state
            if (gamepad1.left_bumper) {
                // Fixed: Cleared stray bracket and aligned parameter to .h
                assist.snapToNearest(pose.h, 4);
            }
            if (gamepad1.right_bumper) {
                assist.release();
            }

            // Monitor auto-scoring trigger state inputs
            boolean wantMacro = gamepad1.right_trigger > 0.5;

            if (wantMacro && !macroActive) {
                // Fixed: Aligned all input parameters to use .h instead of .heading
                macro.followPath(DriverAssist.scoreMacroPath(
                                pose.x, pose.y, pose.h, SCORE_X, SCORE_Y, SCORE_HEADING),
                        pose.h, SCORE_HEADING);
                macroActive = true;
            } else if (!wantMacro) {
                macroActive = false;
            }

            double[] powers;
            if (macroActive) {
                // Fixed: Aligned input to use pose.h
                powers = macro.update(pose.x, pose.y, pose.h, dt);
                telemetry.addLine("MACRO ACTIVE - release trigger for manual control");
            } else {
                // Manual Robot-Centric stick parameters reading loop
                double sx = dead(gamepad1.left_stick_x);
                double sy = dead(-gamepad1.left_stick_y);
                double st = dead(gamepad1.right_stick_x);
                // Fixed: Aligned input to use pose.h
                powers = assist.update(sx, sy, st, pose.h);
            }

            // Distribute calculated powers out to the electrical motor outputs
            setMotorPowers(powers[0], powers[1], powers[2], powers[3]);

            // Feed running metrics down to the driver station telemetry console
            telemetry.addData("Pose", pose.toString());
            telemetry.addData("Heading lock", assist.isLocked()
                    ? String.format("%.0f deg", Math.toDegrees(assist.lockedHeading())) : "off");
            telemetry.update();
        }

        // Safety: Hard stop all wheels upon completion
        setMotorPowers(0, 0, 0, 0);
    }

    /** Filters out minor hardware joystick wiggle drifts */
    private static double dead(double v) {
        return Math.abs(v) < STICK_D ? 0 : v;
    }

    /**
     * Localizer Factory Constructor Stub
     * NOTE: Replace this interface implementation body with your actual active
     * custom sensor module link class when testing on the physical chassis!
     * Example: return new LocalizerAdapters.PinpointLocalizer(hardwareMap, "pinpoint");
     */
    private Localizer createLocalizer() {
        // This used to return a fake localizer whose pose was always (0, 0, 0). Heading never changed,
        // so every heading lock thought the robot was still facing 0 and spun it forever.
        return new LocalizerAdapters.PinpointLocalizer(hardwareMap, "pinpoint");
    }

    // Direct wheel updating pipeline mapping
    private void setMotorPowers(double fl, double fr, double bl, double br) {
        fL.setPower(fl);
        fR.setPower(fr);
        bL.setPower(bl);
        bR.setPower(br);
    }
}