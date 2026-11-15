package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/*
 * Vision Testing
 * An iterative OpMode that tests the VisionSubsystem (the Limelight) and uses it to AIM the turret.
 *   VisionSubsystem  -> reads the Limelight: where is OUR hive's AprilTag cluster?
 *   TurretSubsystem  -> turns the turret (the same subsystem as TurretAlpha)
 * This OpMode owns NO hardware itself. It only reads the gamepads and tells the subsystems what to do.
 *
 * STEP 1: CAMERA ONLY (no turret motion). Do this first.
 *   Just INIT and watch the screen. Check:
 *     - "Limelight" shows a temperature and FPS (it is connected). It takes about 30 seconds to boot.
 *     - "Pipeline" shows the number we want. If not, see APRILTAG_PIPELINE_INDEX in VisionSubsystem.
 *     - Point the turret (by hand) at the hive. "Vision State" must say TARGET_VISIBLE.
 *     - tx is POSITIVE when the target is to the RIGHT of the camera's center, and goes to about
 *       0 when you turn the turret until the camera looks straight at the hive. If it goes the
 *       wrong way, the camera may be mounted rotated. Fix that in the Limelight's web page.
 *     - Try different distances. Where does it stop seeing the tags? Write the number down.
 *     - Try both alliances (START on gamepad 1). Only OUR alliance's tags should be used.
 *
 * STEP 2: AUTO-ALIGN. Only after Step 1 works, and with a hand near the Stop button.
 *   Do the TurretAlpha first test (encoder direction) before this. Auto-align trusts the turret angle.
 *   HOLD gamepad 2's Right Bumper. The turret turns until the target is centered, and follows it.
 *   The turret will not go past its +/-120 degree limits (see TurretSubsystem).
 *
 * GAMEPAD 1:
 *   START -> switch alliance BLUE <-> RED (one switch per press). It starts as BLUE.
 *      (Pairing the controller on the Driver Station is Start + A. If you do that while this
 *       OpMode is running, it also switches the alliance, so check the alliance after pairing.)
 *
 * GAMEPAD 2 (the same turret controls as TurretAlpha and the TeleOp):
 *   Right Bumper (HOLD) -> AUTO-ALIGN the turret to the target
 *   Left Stick X        -> turn the turret by hand (push RIGHT and the turret turns RIGHT). Beats auto-align.
 *   A                   -> STOP the turret (it floats, so you can turn it by hand). Beats everything.
 *   Letting go of the bumper or the stick holds the turret where it is.
 *
 * HOW AUTO-ALIGN WORKS
 *   The camera is ON the turret, so tx is how far the turret still has to turn. But the camera's
 *   tx is positive to the RIGHT, and the turret's angle is positive to the LEFT. So:
 *       new turret target = turret angle NOW - tx
 *   We only pick a new target when the error is bigger than ALIGN_REAIM_THRESHOLD_DEGREES. Picking
 *   a new target every loop would chase the camera's tiny wobble, and the turret's "on target"
 *   answer would keep turning off. If the target is lost, the turret HOLDS its last target.
 *   It does not search.
 *
 * "OK to shoot" for the turret needs BOTH: the camera says tx is small, AND the turret says it
 * is on target. That is the turret's half of the firing gate. (The shooter has the other half.)
 *
 * SAFE USE: the turret moves by itself in Step 2. Keep hands, hair and cables clear of it.
 *
 * Robot configuration (on the Driver Station):
 *   Ethernet Device name: limelight   (VisionSubsystem, Limelight 3A)
 *   Motor name:           turret_motor (TurretSubsystem, with its encoder cable in the same port)
 *
 * HIDDEN FROM THE DRIVER STATION LIST
 *   The camera reading and the auto-align are tested, and TeleOp - Alpha 15 now has auto-align
 *   (ALWAYS-TRACK) built in, so this OpMode is @Disabled to keep the list short.
 *   The code is all still here. To bring it back (for example after moving the camera), put //
 *   in front of the @Disabled line below and redeploy.
 */
@Disabled
@TeleOp(name = "Vision - Alpha", group = "Testing")
public class VisionAlpha extends OpMode {

    // ------------------------------------------------------------------
    // Test-only numbers
    // ------------------------------------------------------------------
    // The turret power, in percent, for the manual stick. A full push turns at this power.
    private static final int TURRET_POWER_PERCENT = 100;
    private static final double PERCENT_PER_ONE = 100.0;

    // The turret stick must move more than this before it counts.
    private static final double TURRET_STICK_DEADBAND = 0.1;

    // Auto-align picks a new turret target only when the camera's tx is bigger than this many
    // degrees. Smaller = more precise but twitchier. The hive window forgives about 5 degrees
    // or more, so there is no need to chase every fraction of a degree.
    private static final double ALIGN_REAIM_THRESHOLD_DEGREES = 1.0;

    // The target counts as "aimed at" when tx is within this many degrees of zero.
    private static final double AIMED_TOLERANCE_DEGREES = 2.0;

    // true = the control map stays on the Driver Station screen while the OpMode RUNS (at the
    // bottom, under the live data). false = it is only shown before START, like it used to be.
    private static final boolean SHOW_CONTROLS_WHILE_RUNNING = true;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private VisionSubsystem visionSubsystem;
    private TurretSubsystem turretSubsystem;

    // What the buttons were doing LAST loop, so one press counts only once.
    private boolean wasStartPressed = false;

    // Remember what the turret was doing last loop, so we hold the angle when it is released.
    private boolean wasTurningTurretManually = false;
    private boolean wasAutoAligning = false;

    /*
     * True only on the loop where a button goes from "not pressed" to "pressed".
     */
    private boolean isNewPress(boolean pressedNow, boolean pressedLastLoop) {
        return pressedNow && !pressedLastLoop;
    }

    /*
     * Adds the button map to telemetry so the drivers can read it.
     */
    private void addControlsTelemetry() {
        telemetry.addLine("--- CONTROLS ---");
        telemetry.addLine("Gamepad 1:  START = switch alliance (BLUE / RED)");
        telemetry.addLine("Gamepad 2:  Right Bumper (hold) = AUTO-ALIGN the turret");
        telemetry.addLine("            Left Stick X = turn turret by hand   A = STOP turret");
    }

    /*
     * START on gamepad 1 switches the alliance. Works before and after the match starts.
     */
    private void checkAllianceToggle() {
        boolean startPressed = gamepad1.start;
        if (isNewPress(startPressed, wasStartPressed)) {
            if (visionSubsystem.getAlliance() == VisionSubsystem.Alliance.BLUE) {
                visionSubsystem.setAlliance(VisionSubsystem.Alliance.RED);
            } else {
                visionSubsystem.setAlliance(VisionSubsystem.Alliance.BLUE);
            }
        }
        wasStartPressed = startPressed;
    }

    /*
     * Turns the turret toward the target. Returns the words to show on the screen.
     * Call it every loop while the Right Bumper is held.
     */
    private String autoAlignTurret() {
        if (!visionSubsystem.hasTarget()) {
            // No target, so there is nothing to aim at. The turret keeps doing what it was doing.
            return "Right Bumper (NO TARGET - holding)";
        }

        double txDegrees = visionSubsystem.getTxDegrees();

        // Pick a new target if the turret is not in POSITION mode yet (the bumper was just
        // pressed), or if the camera says we are far enough off.
        boolean notAimingYet = turretSubsystem.getState() != TurretSubsystem.TurretState.POSITION;
        boolean farOff = Math.abs(txDegrees) > ALIGN_REAIM_THRESHOLD_DEGREES;
        if (notAimingYet || farOff) {
            // tx is positive to the RIGHT, but turret angles are positive to the LEFT, so subtract.
            double newTargetAngle = turretSubsystem.getAngleDegrees() - txDegrees;
            turretSubsystem.setTargetAngle(newTargetAngle);
        }
        return "Right Bumper (AUTO-ALIGN)";
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // The Limelight starts polling here. The turret does not move at INIT.
        visionSubsystem = new VisionSubsystem(hardwareMap);
        turretSubsystem = new TurretSubsystem(hardwareMap);

        telemetry.addData("Status", "Initialized");
        addControlsTelemetry();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        // Step 1 can be done right here: aim the turret by hand and watch the numbers.
        visionSubsystem.update();
        checkAllianceToggle();

        telemetry.addData("Status", "Ready to start!");
        visionSubsystem.addTelemetry(telemetry);

        // The turret is not driven during INIT. We only read it, so you can turn it by hand
        // and watch the angle.
        turretSubsystem.update();
        turretSubsystem.addTelemetry(telemetry);
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
        visionSubsystem.update();

        // 2. Read the gamepads and decide what to do.
        checkAllianceToggle();

        // Stick RIGHT is a positive number on the controller. But turning RIGHT means going
        // clockwise, and clockwise is a NEGATIVE angle. So we flip the sign of the stick.
        double turretStick = -gamepad2.left_stick_x;
        boolean usingTurretStick = Math.abs(turretStick) > TURRET_STICK_DEADBAND;
        boolean autoAlignHeld = gamepad2.right_bumper;
        boolean stopPressed = gamepad2.a;

        // 3. Tell the subsystems what to do, then call their update().
        // The first one that is true wins: STOP, then the stick, then auto-align.
        String turretControl;
        boolean autoAligningNow = false;
        if (stopPressed) {
            turretSubsystem.stop();
            turretControl = "A (STOP)";
        } else if (usingTurretStick) {
            double turretPower = TURRET_POWER_PERCENT / PERCENT_PER_ONE;
            turretSubsystem.setManualPower(turretStick * turretPower);
            turretControl = "Left Stick (MANUAL)";
        } else if (autoAlignHeld) {
            turretControl = autoAlignTurret();
            autoAligningNow = true;
        } else if (wasTurningTurretManually || wasAutoAligning) {
            // The stick or the bumper was just let go, so hold the turret where it stopped.
            turretSubsystem.holdCurrentAngle();
            turretControl = "Released (HOLD)";
        } else {
            // No new command. The turret keeps doing what it was last told to do.
            turretControl = "None (keeps last command)";
        }
        turretSubsystem.update();

        // Remember this loop for the next loop.
        wasTurningTurretManually = usingTurretStick && !stopPressed;
        wasAutoAligning = autoAligningNow;

        // 4. Show what is happening on the Driver Station.
        telemetry.addData("Status", "Run Time: " + runtime.toString());

        visionSubsystem.addTelemetry(telemetry);

        // Is the turret's half of the firing gate open? The camera must say we are centered on
        // the target, AND the turret must say it has reached its target and settled.
        boolean aimed = visionSubsystem.hasTarget()
                && Math.abs(visionSubsystem.getTxDegrees()) <= AIMED_TOLERANCE_DEGREES;
        String aimText;
        if (!visionSubsystem.hasTarget()) {
            aimText = "NO TARGET";
        } else if (aimed && turretSubsystem.isOnTarget()) {
            aimText = "YES - turret is aimed";
        } else if (aimed) {
            aimText = "almost - camera is centered, turret is still settling";
        } else {
            aimText = "NO - off target by " + String.format("%.1f", visionSubsystem.getTxDegrees())
                    + " deg (tx)";
        }
        telemetry.addData("Turret aimed?", aimText);

        turretSubsystem.addTelemetry(telemetry);
        telemetry.addData("Turret Control", turretControl);

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
        // Always leave the motors stopped, and stop polling the camera.
        turretSubsystem.stop();
        visionSubsystem.stop();
    }
}
