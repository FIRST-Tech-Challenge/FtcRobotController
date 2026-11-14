package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/*
 * Drive Forward - Mark 1 (the A-0 Emergency Leave Auto)
 * The simplest Auto there is (rung A-0 in TeamCode/docs/biobuzz-auto-strategy.md). The robot drives
 * straight FORWARD for a fixed time and stops. That is LEAVE = 3 points, as long as the robot is
 * no longer touching the perimeter wall when AUTO ends.
 *
 * WHEN TO USE IT
 *   - Pedro Pathing is not tuned yet (FORESIGHT_TUNED is false), so AutoLeaveAndPark will not
 *     drive. This is the stopgap until it is tuned.
 *   - Or something else is broken at an event (the Pinpoint is not READY, the camera is dead) and
 *     we want to be sure of the 3 LEAVE points.
 *
 * HOW IT WORKS (and why it is only "good enough")
 *   It uses DriveSubsystem, the same class as the TeleOp, and NOTHING else: no Pedro, no
 *   Pinpoint, no camera. It does not know where the robot is. It just sends the same power for the
 *   same time. So the distance depends on the battery (a weaker battery goes less far) and on the
 *   floor, and it does NOT park in the LOADING ZONE. The zone is too far away to hit with a
 *   timer. Parking needs Pedro.
 *
 * WHERE THE ROBOT STARTS
 *   The same start spot as the other Autos: back against the wall, facing the hive. Blue is tile
 *   D6 (top wall, facing down), red is tile C1 (bottom wall, facing up). "Forward" is always
 *   toward the hive, so the same OpMode works for both alliances. Nothing needs to be chosen.
 *   The robot must be inside the 18 inch cube, touching the wall, not in the LOADING ZONE and not
 *   touching a flower. (All of that is in pedro/FieldConstants.java.)
 *
 * HOW TO TUNE IT (all at the top of this file)
 *   1. Start with DRIVE_POWER 0.30 and DRIVE_SECONDS 0.80. Put the robot against the wall, with a
 *      clear floor in front of it (the hive is about 40 inches away on the real field).
 *   2. Run it and measure how far the robot moved. We want about 12 inches: enough to be clearly
 *      off the wall, and far from the hive.
 *   3. Change DRIVE_SECONDS (a bigger number goes farther), run again. Keep DRIVE_POWER low.
 *   4. Run it 5 times in a row with a freshly charged battery, and once with a tired battery,
 *      and write the distances in TeamCode/docs/biobuzz-test-and-match-log.md. If the distance
 *      changes a lot, pick a number that is still safe with the tired battery.
 *
 * Nothing moves during INIT (rule G403). The robot only moves after START. Stop it any time with
 * the Driver Station STOP button.
 */
@Autonomous(name = "Drive Forward - Mark 1", group = "Auto",
        preselectTeleOp = "TeleOp - Alpha 14")
public class AutoEmergencyLeave extends OpMode {

    // ------------------------------------------------------------------
    // Settings (change these to change the Auto)
    // ------------------------------------------------------------------
    // How hard to drive, from 0.0 to 1.0. Keep it low: a tall robot that stops from full speed can
    // tip, and a low power is easier to repeat. 0.30 is a starting guess.
    private static final double DRIVE_POWER = 0.30;

    // How long to drive, in seconds. 0.80 is a starting guess. Tune it by measuring (see above).
    private static final double DRIVE_SECONDS = 0.80;

    // ------------------------------------------------------------------
    // The steps of this Auto. The robot is always in exactly one of these.
    // ------------------------------------------------------------------
    private enum AutoState {
        DRIVING,
        DONE
    }

    // ------------------------------------------------------------------
    // Things this OpMode uses
    // ------------------------------------------------------------------
    private DriveSubsystem driveSubsystem;

    private AutoState autoState = AutoState.DONE;

    // Times the drive. (A timer is used instead of sleep() so loop() keeps running.)
    private final ElapsedTime driveTimer = new ElapsedTime();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // The wheels start stopped. Nothing moves in INIT.
        driveSubsystem = new DriveSubsystem(hardwareMap);

        telemetry.addLine("Ready. Put the robot on its start spot against the wall, then press START.");
        addSettingsTelemetry();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        addSettingsTelemetry();
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        driveTimer.reset();
        autoState = AutoState.DRIVING;
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        // 1. (No sensors in this Auto.)

        // 2. Decide what to do next, and tell the drivetrain.
        switch (autoState) {
            case DRIVING:
                // forward = DRIVE_POWER, strafe = 0, turn = 0: straight ahead.
                driveSubsystem.drive(DRIVE_POWER, 0.0, 0.0);
                if (driveTimer.seconds() >= DRIVE_SECONDS) {
                    driveSubsystem.stop();
                    autoState = AutoState.DONE;
                }
                break;

            case DONE:
                // Keep telling the wheels to stop, so nothing can drift.
                driveSubsystem.stop();
                break;
        }

        // 3. Telemetry
        telemetry.addData("State", autoState);
        telemetry.addData("Time driving (s)", "%.2f of %.2f", driveTimer.seconds(), DRIVE_SECONDS);
        addSettingsTelemetry();
        driveSubsystem.addTelemetry(telemetry);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        driveSubsystem.stop();
    }

    // ------------------------------------------------------------------
    // Telemetry helpers
    // ------------------------------------------------------------------
    private void addSettingsTelemetry() {
        telemetry.addData("Drive power", "%.2f", DRIVE_POWER);
        telemetry.addData("Drive seconds", "%.2f", DRIVE_SECONDS);
    }
}
