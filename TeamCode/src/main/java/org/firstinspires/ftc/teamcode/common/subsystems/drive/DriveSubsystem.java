package org.firstinspires.ftc.teamcode.common.subsystems.drive;

import org.firstinspires.ftc.teamcode.common.hardware.DriveHardware;
import org.firstinspires.ftc.teamcode.core.fsm.FSM;
import org.firstinspires.ftc.teamcode.core.fsm.Transition;
import org.firstinspires.ftc.teamcode.core.robot.Subsystem;

/**
 * Controls a mecanum drivetrain through a small finite-state machine.
 *
 * <p>Call the request methods from a robot public API or another allowed higher layer. The
 * subsystem applies requests during its next {@link #update()} call.</p>
 */
public class DriveSubsystem implements Subsystem {
    private enum RequestedMode {
        DISABLED,
        MANUAL,
        HEADING_HOLD
    }

    private final DriveHardware driveHardware;
    private final DisabledDriveState disabledState;
    private final ManualDriveState manualDriveState;
    private final HeadingHoldState headingHoldState;
    private final FSM fsm;

    private RequestedMode requestedMode = RequestedMode.DISABLED;
    private double requestedForward;
    private double requestedStrafe;
    private double requestedRotate;

    /** Creates a drivetrain subsystem that sends all motor commands through {@code driveHardware}. */
    public DriveSubsystem(DriveHardware driveHardware) {
        if (driveHardware == null) {
            throw new IllegalArgumentException("Drive subsystem needs drive hardware.");
        }

        this.driveHardware = driveHardware;
        disabledState = new DisabledDriveState(this);
        manualDriveState = new ManualDriveState(this);
        headingHoldState = new HeadingHoldState(this);
        fsm = new FSM(disabledState);

        addTransitions();
    }

    @Override
    public void initialize() {
        fsm.initialize();
    }

    @Override
    public void update() {
        fsm.update();
    }

    @Override
    public void stop() {
        requestedMode = RequestedMode.DISABLED;
        requestedForward = 0.0;
        requestedStrafe = 0.0;
        requestedRotate = 0.0;
        driveHardware.stop();
    }

    @Override
    public String getName() {
        return "Drive";
    }

    /** Stores the translation and rotation request to apply during the next update. */
    public void drive(double forward, double strafe, double rotate) {
        requestedForward = safeInput(forward);
        requestedStrafe = safeInput(strafe);
        requestedRotate = safeInput(rotate);
    }

    /** Requests normal driver-controlled mecanum behavior. */
    public void enableManualDrive() {
        requestedMode = RequestedMode.MANUAL;
    }

    /** Requests the safe disabled behavior. */
    public void disableDrive() {
        requestedMode = RequestedMode.DISABLED;
    }

    /** Requests the heading-hold placeholder behavior. */
    public void enableHeadingHold() {
        requestedMode = RequestedMode.HEADING_HOLD;
    }

    /** Returns the active state name, or the initial disabled state name before initialization. */
    public String getCurrentStateName() {
        String currentStateName = fsm.getCurrentStateName();
        return currentStateName == null ? disabledState.getName() : currentStateName;
    }

    public double getRequestedForward() {
        return requestedForward;
    }

    public double getRequestedStrafe() {
        return requestedStrafe;
    }

    public double getRequestedRotate() {
        return requestedRotate;
    }

    void applyRequestedMecanumDrive() {
        double frontLeft = requestedForward + requestedStrafe + requestedRotate;
        double frontRight = requestedForward - requestedStrafe - requestedRotate;
        double rearLeft = requestedForward - requestedStrafe + requestedRotate;
        double rearRight = requestedForward + requestedStrafe - requestedRotate;

        double largestMagnitude = Math.max(Math.abs(frontLeft), Math.abs(frontRight));
        largestMagnitude = Math.max(largestMagnitude, Math.abs(rearLeft));
        largestMagnitude = Math.max(largestMagnitude, Math.abs(rearRight));

        if (largestMagnitude > 1.0) {
            frontLeft /= largestMagnitude;
            frontRight /= largestMagnitude;
            rearLeft /= largestMagnitude;
            rearRight /= largestMagnitude;
        }

        driveHardware.setMotorPowers(frontLeft, frontRight, rearLeft, rearRight);
    }

    void stopDrive() {
        driveHardware.stop();
    }

    private void addTransitions() {
        fsm.addTransition(new Transition(disabledState, manualDriveState,
                () -> requestedMode == RequestedMode.MANUAL));
        fsm.addTransition(new Transition(disabledState, headingHoldState,
                () -> requestedMode == RequestedMode.HEADING_HOLD));
        fsm.addTransition(new Transition(manualDriveState, disabledState,
                () -> requestedMode == RequestedMode.DISABLED));
        fsm.addTransition(new Transition(manualDriveState, headingHoldState,
                () -> requestedMode == RequestedMode.HEADING_HOLD));
        fsm.addTransition(new Transition(headingHoldState, disabledState,
                () -> requestedMode == RequestedMode.DISABLED));
        fsm.addTransition(new Transition(headingHoldState, manualDriveState,
                () -> requestedMode == RequestedMode.MANUAL));
    }

    private double safeInput(double input) {
        if (Double.isNaN(input) || Double.isInfinite(input)) {
            return 0.0;
        }
        return input;
    }
}
