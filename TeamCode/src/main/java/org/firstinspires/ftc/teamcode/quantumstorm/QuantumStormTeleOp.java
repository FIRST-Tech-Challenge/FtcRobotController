package org.firstinspires.ftc.teamcode.quantumstorm;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.quantumstorm.DriveChain.MecanumDrive;
import org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms.FlowerIntake;
import org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms.SpindexerControls;

@TeleOp(name = Constants.TELEOP_NAME, group = "Quantum Storm")
public class QuantumStormTeleOp extends LinearOpMode {

    // Drive chain
    private MecanumDrive drive;

    // Mechanism motors
    private DcMotor intake;
    private DcMotorEx shooter;

    // CR servos
    private CRServo shooterServo;
    private final SpindexerControls spindexerControls = new SpindexerControls();
    private final org.firstinspires.ftc.teamcode.biobuzz.FeedCadence feedCadence=new org.firstinspires.ftc.teamcode.biobuzz.FeedCadence(Constants.SPINDEXER_FEED_PULSE_SEC,Constants.SPINDEXER_SHOT_GAP_SEC);

    // Flower intake arm (position servo)
    private FlowerIntake flowerIntake;

    // Intake:
    //  1 = forward
    //  0 = stopped
    // -1 = reverse
    private int intakeState = 0;

    private boolean shooterRunning = false;

    // Require a fresh A press for each intake direction command.
    private boolean previousA = false;

    // Adjustable fraction of SHOOTER_BASE_RPM.
    private double shooterPower = Constants.SHOOTER_POWER;

    // Track even during controller assignment so releasing Start cannot start a shooter.
    private boolean previousGamepad2A = false;

    // For the shoulder buttons and triggers used as speed steps
    private boolean previousLeftBumper = false;
    private boolean previousLeftTrigger = false;

    // Multiplier before the most recent change, for tuning telemetry
    private double previousShooterPower = Constants.SHOOTER_POWER;

    @Override
    public void runOpMode() {

        // =========================================================
        // HARDWARE MAP
        // =========================================================

        drive = new MecanumDrive(hardwareMap);

        intake = hardwareMap.get(DcMotor.class, Constants.INTAKE_MOTOR);

        shooter = hardwareMap.get(DcMotorEx.class, Constants.SHOOTER_MOTOR);

        shooterServo = hardwareMap.get(CRServo.class, Constants.SHOOTER_SERVO);

        flowerIntake = new FlowerIntake(hardwareMap);


        // =========================================================
        // MOTOR DIRECTIONS
        // =========================================================

        intake.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // =========================================================
        // SERVO DIRECTIONS
        // =========================================================

        // D-pad UP = spindexer forward; DOWN = spindexer reverse, matching autonomous.
        shooterServo.setDirection(CRServo.Direction.REVERSE);


        // =========================================================
        // ZERO POWER BEHAVIOR
        // =========================================================

        intake.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        shooter.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // =========================================================
        // START EVERYTHING STOPPED
        // =========================================================

        stopAll();

        // The flower arm moves to UP as soon as INIT is pressed
        flowerIntake.lift();


        telemetry.addLine("Quantum Storm Two Driver Ready");
        telemetry.addLine("");

        telemetry.addLine("GAMEPAD 1 - DRIVE + INTAKE");
        telemetry.addLine("Left Stick = Drive / Strafe");
        telemetry.addLine("Right Stick X = Turn");
        telemetry.addLine("A (without BACK/START) = Intake Forward ON");
        telemetry.addLine("B = Intake OFF (also with START held)");
        telemetry.addLine("Hold BACK + tap A = Intake Reverse ON (release START)");
        telemetry.addLine("START + A = Assign gamepad 1; no intake direction command");

        telemetry.addLine("");

        telemetry.addLine("GAMEPAD 2 - SHOOTER + SPINDEXER");
        telemetry.addLine("A = Shooter 1 START, B = Shooter 1 STOP");
        telemetry.addLine("Hold BACK + tap LB = Shooter RPM +60 (multiplier +0.01)");
        telemetry.addLine("Hold BACK + tap LT = Shooter RPM -60 (multiplier -0.01)");
        telemetry.addLine("Release START before using shooter START/STOP buttons");
        telemetry.addLine("Hold D-Pad UP = Spindexer Forward (pulsed)");
        telemetry.addLine("Hold D-Pad DOWN = Spindexer Reverse");
        telemetry.addLine("Release D-Pad = Spindexer STOP");
        telemetry.addLine("BACK + D-Pad DOWN = Flower Intake DROP (-135 deg)");
        telemetry.addLine("BACK + D-Pad UP = Flower Intake UP (0 deg)");
        telemetry.addLine("LB + D-Pad UP/DOWN = Flower Angle Nudge");
        telemetry.addLine("After flower controls, release D-Pad before running spindexer");

        telemetry.update();


        waitForStart();

        // Start+A assigns gamepad 1 in the Driver Station. A held through INIT/START
        // must be released before it can command the intake during TeleOp.
        previousA = gamepad1.a;


        // =========================================================
        // MAIN LOOP
        // =========================================================

        while (opModeIsActive()) {

            // =====================================================
            // GAMEPAD 1 - MECANUM DRIVE
            // =====================================================

            // This robot's tested forward/backward convention is inverted.
            // Correct only TeleOp's Y input; keep strafe, turn and autonomous unchanged.
            drive.drive(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x);


            // =====================================================
            // GAMEPAD 1 - INTAKE START / STOP
            // =====================================================

            boolean currentA = gamepad1.a;
            boolean currentB = gamepad1.b;


            // Fresh A selects forward; holding Back at that press selects reverse.
            // Start+A is reserved for Driver Station assignment. Track A even while
            // Start is held so releasing Start cannot issue an intake command.
            // Changing Back while A is held does not change direction.
            if (!gamepad1.start && currentA && !previousA) {
                intakeState = gamepad1.back ? -1 : 1;
            }
            // STOP always wins, including A+B, Back+A+B and Start+A+B.
            if (currentB) intakeState = 0;
            previousA = currentA;


            // =====================================================
            // INTAKE MOTOR
            // =====================================================

            double intakeOutput =
                    intakeState * Constants.INTAKE_POWER;

            intake.setPower(intakeOutput);


            // =====================================================
            // GAMEPAD 2 - FLOWER INTAKE SERVO
            // =====================================================

            // Gamepad 2: BACK + D-pad UP/DOWN = lift/drop, LB + D-pad = nudge
            flowerIntake.handleGamepad(gamepad2);

            // A flower-arm press stays reserved until the D-pad is released,
            // even if BACK/LB is released first.
            int spindexerDemand = spindexerControls.update(
                    gamepad2.dpad_up, gamepad2.dpad_down, gamepad2.back, gamepad2.left_bumper);
            boolean spindexerDpadUp = spindexerDemand > 0;
            boolean spindexerDpadDown = spindexerDemand < 0;


            // =====================================================
            // GAMEPAD 2 - SHOOTER MOTOR
            // =====================================================

            // Triggers are analog, so treat them as pressed past a threshold
            boolean currentLeftBumper = gamepad2.left_bumper;
            boolean currentLeftTrigger =
                    gamepad2.left_trigger > Constants.TRIGGER_THRESHOLD;


            // Start+A/B assigns controllers in the Driver Station, not shooter commands.
            // Require a fresh A press after assignment; STOP wins if both are pressed.
            if (!gamepad2.start) {
                if (gamepad2.a && !previousGamepad2A) shooterRunning = true;
                if (gamepad2.b) shooterRunning = false;
            }


            // Require Back already held when a shoulder button is freshly pressed.
            // Track raw buttons below even while locked: holding LB then pressing Back
            // must not accidentally adjust RPM. Opposite requests cancel each other.
            if (gamepad2.back && currentLeftBumper && !previousLeftBumper && !currentLeftTrigger) {
                previousShooterPower = shooterPower;
                shooterPower = clampShooterPower(shooterPower + Constants.SHOOTER_POWER_INCREMENT);
            }

            // Back + LT = shooter RPM multiplier down
            if (gamepad2.back && currentLeftTrigger && !previousLeftTrigger && !currentLeftBumper) {
                previousShooterPower = shooterPower;
                shooterPower = clampShooterPower(shooterPower - Constants.SHOOTER_POWER_INCREMENT);
            }


            previousGamepad2A = gamepad2.a;
            previousLeftBumper = currentLeftBumper;
            previousLeftTrigger = currentLeftTrigger;


            if (shooterRunning) {

                shooter.setPower(shooterPower);

            } else {

                shooter.setPower(0);
            }


            // =====================================================
            // GAMEPAD 2 - SPINDEXER CR SERVO
            // =====================================================
            //
            // UP = FORWARD
            // DOWN = REVERSE
            //
            // Servo direction is configured above to match
            // your actual physical mechanism.
            // =====================================================

            if (spindexerDpadUp) {

                shooterServo.setPower(feedCadence.apply(1,System.nanoTime()/1e9)*Constants.SHOOTER_SERVO_POWER);

            } else if (spindexerDpadDown) {

                shooterServo.setPower(feedCadence.apply(-1,System.nanoTime()/1e9)*Constants.SHOOTER_SERVO_POWER);

            } else {

                feedCadence.apply(0,System.nanoTime()/1e9);
                shooterServo.setPower(0);
            }


            // =====================================================
            // TELEMETRY
            // =====================================================

            telemetry.addLine("=== DRIVER 1 ===");

            drive.addTelemetry(telemetry);


            if (intakeState == 1) {

                telemetry.addData(
                        "Intake",
                        "FORWARD");

            } else if (intakeState == -1) {

                telemetry.addData(
                        "Intake",
                        "REVERSE");

            } else {

                telemetry.addData(
                        "Intake",
                        "STOPPED");
            }

            telemetry.addLine("");
            telemetry.addLine("=== DRIVER 2 ===");
            telemetry.addData("Shooter power adjustment", gamepad2.back ? "UNLOCKED: tap LB/LT" : "LOCKED: hold BACK");


            telemetry.addData(
                    "Shooter 1",
                    "%s | power %.2f",
                    shooterRunning ? "ON" : "OFF",
                    shooterPower);

            telemetry.addData("Shooter applied / requested power", "%.2f / %.2f",
                    shooter.getPower(), shooterRunning ? shooterPower : 0);

            // Previous -> current, so a useful RPM multiplier can be written down
            telemetry.addData(
                    "Shooter power",
                    "%.2f -> %.2f",
                    previousShooterPower,
                    shooterPower);


            telemetry.addData(
                    "Flower Intake",
                    "%s  %.0f deg  (servo pos %.3f)",
                    flowerIntake.isDown() ? "DOWN" : "UP",
                    flowerIntake.getAngle(),
                    flowerIntake.getPosition());


            if (spindexerDpadUp) {

                telemetry.addData(
                        "Spindexer",
                        "FORWARD");

            } else if (spindexerDpadDown) {

                telemetry.addData(
                        "Spindexer",
                        "REVERSE");

            } else {

                telemetry.addData(
                        "Spindexer",
                        "STOPPED");
            }


            telemetry.update();
        }


        // =========================================================
        // STOP EVERYTHING
        // =========================================================

        stopAll();
    }

    private void stopAll() {

        drive.stop();

        intake.setPower(0);
        shooter.setPower(0);

        feedCadence.apply(0,System.nanoTime()/1e9);
        shooterServo.setPower(0);
    }

    /**
     * Keeps the adjustable RPM multiplier inside the configured bounds.
     * The range spans both signs so tuning can reverse the motor if needed.
     */
    private double clampShooterPower(double power) {

        if (power < Constants.SHOOTER_POWER_MIN) {
            return Constants.SHOOTER_POWER_MIN;
        }

        if (power > Constants.SHOOTER_POWER_MAX) {
            return Constants.SHOOTER_POWER_MAX;
        }

        // Keep button steps on the configured grid, including an exact zero target.
        double step = Constants.SHOOTER_POWER_INCREMENT;
        return Math.round(power / step) * step;
    }
}
