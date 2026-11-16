package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterServoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

/*
 * Alpha-Shooter Testing
 * An iterative OpMode that tests the whole ball path with three subsystems:
 *   ShooterSubsystem       -> the FLYWHEEL and the HOOD WHEEL (plain POWER, no speed control)
 *   ShooterServoSubsystem  -> the shooter GATE servo
 *   IntakeSubsystem        -> the INTAKE (which also runs the transfer)
 * This OpMode owns NO hardware itself. It only reads the gamepads and tells the subsystems
 * what to do. (ShooterRPMAlpha is the speed-control version of the wheel tests.)
 *
 * TWO CONTROLLERS: the shooter and gate are on gamepad1, the intake is on gamepad2.
 * That keeps the buttons from fighting each other, because both jobs want the bumpers and D-pad.
 *
 * ------------------------- GAMEPAD 1: SHOOTER + GATE -------------------------
 * HOLD a button to run BOTH the flywheel and the hood wheel.
 * Each motor has its OWN power number for each button, so you can tune them
 * separately. Starting values (flywheel / hood wheel):
 *   Y  -> 60% / 60%   (see the note below)
 *   X  -> 55% / 55%   (see the note below)
 *   No button -> both coast to a stop
 * These two were the closest shots in testing with BOTH wheels on 6000 RPM motors.
 *
 * NOTE: THE FLYWHEEL IS NOW THE 1620 RPM MOTOR. The same POWER now spins the flywheel about
 * 3.7 times slower (60% is about 970 RPM with no load, not 3600), so these power shots are NOT
 * the same shots any more. The hood wheel is still the 6000 RPM motor (60% -> 3600 RPM, 55% ->
 * 3300 RPM with no load). To see the real speeds, hold Y or X and read "Flywheel RPM" and
 * "Hood Wheel RPM" in the shooter section of the telemetry. With a ball and the wheel's own
 * drag the real speed is LOWER than no load, and it drops as the battery drains.
 * For shots, use ShooterRPMAlpha (speed control in RPM) instead of these power shots.
 *
 * REVERSE (for clearing a jam, 60% power the other way):
 *   Right Bumper -> reverses the FLYWHEEL only
 *   Left Bumper  -> reverses the HOOD WHEEL only
 *
 * A reverse bumper beats the shooter buttons for that motor.
 * If more than one shooter button is held, the HIGHEST power wins.
 *
 * SHOOTER GATE (the servo), separate from the wheel buttons above:
 *   Left Trigger -> TOGGLES the gate between CLOSED and OPENED (one flip per press)
 *   D-pad Up     -> nudge the gate's CURRENT position toward 1.0 (small step)
 *   D-pad Down   -> nudge the gate's CURRENT position toward 0.0 (small step)
 * The gate positions are not measured yet. Use the D-pad nudge to creep the gate to the
 * right spot for each state, read the number on the screen, and then type it into
 * CLOSED_POSITION / OPENED_POSITION / LIFTED_POSITION in ShooterServoSubsystem.
 * The gate starts CLOSED when you hit INIT. It does NOT depend on the wheels.
 * (This OpMode can't reach the gate's third position, LIFTED. Shooter RPM - Alpha 2 can.)
 *
 * ------------------------- GAMEPAD 2: INTAKE -------------------------
 * HOLD a bumper to run the intake. There is ONE power dial, and the D-pad turns it up and down.
 *   Right Bumper -> INTAKE (pull balls in) at the dial power
 *   Left Bumper  -> REVERSE (push balls out) at the dial power
 *   D-pad Up     -> turn the dial UP by 5% (each press)
 *   D-pad Down   -> turn the dial DOWN by 5% (each press)
 *   No bumper    -> intake stops
 * Left Bumper beats Right Bumper, so clearing a jam always works.
 *
 * THE POWER DIAL IS FOR TESTING ONLY. When this test shows us the best number, change
 * DEFAULT_INTAKE_POWER at the top of IntakeSubsystem.
 *
 * THE INTAKE ALSO RUNS THE TRANSFER. One motor spins the front intake AND a hex shaft with
 * grippers in the transfer chute. Test with a few balls loaded in the chute.
 * KEEP HANDS AND BALLS CLEAR OF THE FLYWHEEL when you run the intake and shooter together.
 *
 * On a PS5 controller: A = Cross, B = Circle, X = Square, Y = Triangle, bumpers = L1 / R1.
 *
 * Robot configuration (on the Driver Station). The NAMES live in each subsystem class,
 * so check them there if they ever change:
 *   Motor name: flywheel      (ShooterSubsystem,       goBILDA 5202/3/4 series)
 *   Motor name: hood_wheel     (ShooterSubsystem,       goBILDA 5202/3/4 series)
 *   Motor name: intake        (IntakeSubsystem,        goBILDA 5202/3/4 series)
 *   Servo name: shooter_servo (ShooterServoSubsystem,  goBILDA dual mode servo, POSITIONAL mode)
 * The motor directions are set inside the subsystems, not in this file.
 *
 * HIDDEN FROM THE DRIVER STATION LIST
 *   The shooter now runs on speed control (RPM), and ShooterRPMAlpha and TeleOp - Alpha 15 cover
 *   everything this open-loop POWER test did, so this OpMode is @Disabled to keep the list short.
 *   The code is all still here. To bring it back (for example for the raw ticks/sec jitter test
 *   in issue #43), put // in front of the @Disabled line below and redeploy.
 */
@Disabled
@TeleOp(name = "Shooter Power - Alpha", group = "Testing")
public class ShooterPowerAlpha extends OpMode {

    // ------------------------------------------------------------------
    // Gamepad 1: shooter wheels
    // ------------------------------------------------------------------
    // Each button sets the FLYWHEEL and the HOOD WHEEL power (0.0 to 1.0).
    // They are separate numbers on purpose, so you can tune each wheel on its own.
    // Shooting power is always POSITIVE. Change these to test different speeds.
    //
    //                                Y     X
    private static final double FLYWHEEL_POWER_Y = 0.60;
    private static final double FLYWHEEL_POWER_X = 0.55;

    // The hood wheel matches the flywheel here. Ball spin comes from the DIFFERENCE between how
    // fast the two wheel surfaces move, and surface speed = RPM x wheel diameter. The flywheel
    // is about 4 inches and the hood wheel about 2 inches, so at the SAME power the flywheel
    // surface moves about TWICE as fast. For a no-spin shot, the flywheel needs about HALF the
    // hood wheel's power (for example 45% flywheel with 90% hood wheel). To reduce spin, raise
    // the hood wheel power or lower the flywheel. Lowering the hood wheel makes the spin worse.
    private static final double HOOD_WHEEL_POWER_Y = 0.60;
    private static final double HOOD_WHEEL_POWER_X = 0.55;

    // Reverse power is NEGATIVE so the motor spins the other way.
    private static final double FLYWHEEL_POWER_REVERSE = -0.60;
    private static final double HOOD_WHEEL_POWER_REVERSE = -0.60;

    // ------------------------------------------------------------------
    // Gamepad 1: shooter gate
    // ------------------------------------------------------------------
    // The triggers give a number from 0.0 to 1.0. Past this much counts as "pressed".
    private static final double TRIGGER_PRESSED_THRESHOLD = 0.5;

    // How far one D-pad press moves the gate servo position (the position runs 0.0 to 1.0).
    // 0.01 is about 3 degrees on a 300 degree servo (0.01 x 300). Make it smaller for
    // finer adjustment, or bigger to move faster.
    private static final double GATE_NUDGE_STEP = 0.01;

    // ------------------------------------------------------------------
    // Gamepad 2: intake power dial
    // ------------------------------------------------------------------
    // How much one D-pad press changes the dial, in percent. 5 means 5%.
    private static final int DIAL_STEP_PERCENT = 5;

    // The lowest and highest the dial can go, in percent.
    private static final int DIAL_MIN_PERCENT = 0;
    private static final int DIAL_MAX_PERCENT = 100;

    // ------------------------------------------------------------------
    // true = the control map stays on the Driver Station screen while the OpMode RUNS (at the
    // bottom, under the live data). false = it is only shown before START, like it used to be.
    private static final boolean SHOW_CONTROLS_WHILE_RUNNING = true;

    // Declare OpMode members.
    // ------------------------------------------------------------------
    private ElapsedTime runtime = new ElapsedTime();

    // Each subsystem owns its own hardware. This OpMode only calls their methods.
    private ShooterSubsystem shooterSubsystem;
    private ShooterServoSubsystem shooterServoSubsystem;
    private IntakeSubsystem intakeSubsystem;

    // The intake dial in WHOLE percent (50 means 50%). We count in whole numbers so adding 5
    // over and over never drifts to something like 0.4999999.
    private int dialPercent;

    // What the buttons were doing LAST loop. A press lasts many loops (about 50 per second),
    // but we want it to count only ONCE. So we only act when it was NOT pressed last loop
    // and IS pressed now.
    private boolean wasGateTogglePressed = false;
    private boolean wasGateNudgeUpPressed = false;
    private boolean wasGateNudgeDownPressed = false;
    private boolean wasDialUpPressed = false;
    private boolean wasDialDownPressed = false;

    /*
     * Adds the button map to telemetry so the drivers can read it.
     */
    private void addControlsTelemetry() {
        telemetry.addLine("--- GAMEPAD 1: SHOOTER (flywheel / hood wheel) ---");
        // Each line shows the flywheel power / hood wheel power for that button.
        telemetry.addLine(String.format("Y: %.0f%% / %.0f%%   X: %.0f%% / %.0f%%",
                FLYWHEEL_POWER_Y * 100, HOOD_WHEEL_POWER_Y * 100,
                FLYWHEEL_POWER_X * 100, HOOD_WHEEL_POWER_X * 100));
        telemetry.addLine("(flywheel / hood wheel)");
        telemetry.addLine("Right Bumper = REVERSE flywheel only");
        telemetry.addLine("Left Bumper = REVERSE hood wheel only");
        telemetry.addLine("--- GAMEPAD 1: GATE ---");
        telemetry.addLine("Left Trigger = TOGGLE gate (closed / opened)");
        telemetry.addLine("D-pad Up / Down = nudge gate position +/-");
        telemetry.addLine("--- GAMEPAD 2: INTAKE ---");
        telemetry.addLine("Right Bumper = INTAKE   Left Bumper = REVERSE");
        telemetry.addLine("D-pad Up / Down = power dial +5% / -5%");
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Each subsystem gets its own hardware from the hardware map.
        shooterSubsystem = new ShooterSubsystem(hardwareMap);
        // The gate servo moves to CLOSED here, so it starts from a known place.
        shooterServoSubsystem = new ShooterServoSubsystem(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);

        // Start the dial wherever the subsystem starts it, so the number only lives in one place.
        // Math.round gives the nearest whole number (0.5 * 100 = 50).
        dialPercent = (int) Math.round(intakeSubsystem.getIntakePower() * 100);

        // Display initialization status on Driver Station
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Encoder Status", "Wheel encoders may be NOT CONNECTED - power control only");
        addControlsTelemetry();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        // The Driver Station clears its screen every loop, so we add the
        // button map again here. That keeps it on the screen until START.
        telemetry.addData("Status", "Ready to start!");
        telemetry.addData("Intake power dial", "%d%%", dialPercent);
        addControlsTelemetry();
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        // 1. Read sensors.
        // (The shooter reads its own encoders inside shooterSubsystem.update(), below.)
        intakeSubsystem.update();

        // 2. Read the gamepads and decide what to do.

        // ---- Gamepad 1: shooter wheels ----
        double flywheelPower;
        double hoodWheelPower;
        String shooterControl;

        // Check the shooter buttons from HIGHEST power to LOWEST.
        // The first one that is pressed wins, and the rest are skipped.
        // Each button sets the flywheel power AND the hood wheel power.
        if (gamepad1.y) {
            flywheelPower = FLYWHEEL_POWER_Y;
            hoodWheelPower = HOOD_WHEEL_POWER_Y;
            shooterControl = "Y";
        } else if (gamepad1.x) {
            flywheelPower = FLYWHEEL_POWER_X;
            hoodWheelPower = HOOD_WHEEL_POWER_X;
            shooterControl = "X";
        } else {
            // No button held, so stop both motors.
            flywheelPower = 0.0;
            hoodWheelPower = 0.0;
            shooterControl = "None";
        }
        String flywheelControl = shooterControl;
        String hoodWheelControl = shooterControl;

        // REVERSE beats the shooter buttons, so clearing a jam always works.
        if (gamepad1.right_bumper) {
            flywheelPower = FLYWHEEL_POWER_REVERSE;
            flywheelControl = "Right Bumper (REVERSE)";
        }
        if (gamepad1.left_bumper) {
            hoodWheelPower = HOOD_WHEEL_POWER_REVERSE;
            hoodWheelControl = "Left Bumper (REVERSE)";
        }

        // ---- Gamepad 1: shooter gate ----
        // A button counts as ONE press only on the loop where it goes from "not pressed"
        // to "pressed". Without this check, holding it would flip the gate every loop.
        boolean gateTogglePressed = gamepad1.left_trigger > TRIGGER_PRESSED_THRESHOLD;
        boolean gateNudgeUpPressed = gamepad1.dpad_up;
        boolean gateNudgeDownPressed = gamepad1.dpad_down;

        // ---- Gamepad 2: intake ----
        boolean dialUpPressed = gamepad2.dpad_up;
        boolean dialDownPressed = gamepad2.dpad_down;

        // Turn the dial only on the loop where the button FIRST goes down.
        if (dialUpPressed && !wasDialUpPressed) {
            dialPercent = dialPercent + DIAL_STEP_PERCENT;
        }
        if (dialDownPressed && !wasDialDownPressed) {
            dialPercent = dialPercent - DIAL_STEP_PERCENT;
        }

        // Keep the dial between its lowest and highest settings.
        if (dialPercent > DIAL_MAX_PERCENT) {
            dialPercent = DIAL_MAX_PERCENT;
        }
        if (dialPercent < DIAL_MIN_PERCENT) {
            dialPercent = DIAL_MIN_PERCENT;
        }

        // 3. Tell the subsystems what to do, then call their update().

        // Shooter wheels. setPower() is plain power with no speed control. Zero power on
        // BOTH wheels is the same as stop(), so they coast down.
        shooterSubsystem.setPower(flywheelPower, hoodWheelPower);
        shooterSubsystem.update();

        // Gate.
        if (gateTogglePressed && !wasGateTogglePressed) {
            shooterServoSubsystem.toggle();
        }
        if (gateNudgeUpPressed && !wasGateNudgeUpPressed) {
            shooterServoSubsystem.nudgePosition(GATE_NUDGE_STEP);
        }
        if (gateNudgeDownPressed && !wasGateNudgeDownPressed) {
            shooterServoSubsystem.nudgePosition(-GATE_NUDGE_STEP);
        }

        // Intake. Turn the percent into the 0.0 to 1.0 number the subsystem wants (50 -> 0.50).
        intakeSubsystem.setIntakePower(dialPercent / 100.0);

        // Check REVERSE first, so clearing a jam always works.
        String intakeControl;
        if (gamepad2.left_bumper) {
            intakeSubsystem.reverse();
            intakeControl = "Left Bumper (REVERSE)";
        } else if (gamepad2.right_bumper) {
            intakeSubsystem.intake();
            intakeControl = "Right Bumper (INTAKE)";
        } else {
            // No bumper held, so stop the intake.
            intakeSubsystem.stop();
            intakeControl = "None";
        }

        // Remember this loop's buttons for the next loop.
        wasGateTogglePressed = gateTogglePressed;
        wasGateNudgeUpPressed = gateNudgeUpPressed;
        wasGateNudgeDownPressed = gateNudgeDownPressed;
        wasDialUpPressed = dialUpPressed;
        wasDialDownPressed = dialDownPressed;

        // 4. Show what is happening on the Driver Station.
        // The power shown is the power we COMMANDED, so it is negative when reversing.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Flywheel Control", flywheelControl);
        telemetry.addData("Flywheel Power", "%.0f%%", flywheelPower * 100);
        telemetry.addData("Hood Wheel Control", hoodWheelControl);
        telemetry.addData("Hood Wheel Power", "%.0f%%", hoodWheelPower * 100);
        telemetry.addData("Intake Control", intakeControl);
        shooterSubsystem.addTelemetry(telemetry);
        shooterServoSubsystem.addTelemetry(telemetry);
        intakeSubsystem.addTelemetry(telemetry);

        // 5. Keep the button map on the screen while running, at the bottom under the live data.
        //    (Before START the same map is shown by init() and init_loop().)
        if (SHOW_CONTROLS_WHILE_RUNNING) {
            addControlsTelemetry();
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        // Always leave the motors stopped.
        // The gate servo just stays where it is. A servo holds its position by itself.
        shooterSubsystem.stop();
        intakeSubsystem.stop();
    }
}
