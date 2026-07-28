package org.firstinspires.ftc.teamcode.common.subsystems.intake;

import org.firstinspires.ftc.teamcode.common.hardware.IntakeHardware;
import org.firstinspires.ftc.teamcode.core.fsm.FSM;
import org.firstinspires.ftc.teamcode.core.fsm.Transition;
import org.firstinspires.ftc.teamcode.core.robot.Subsystem;

/** Controls the optional intake through a small requested-mode finite-state machine. */
public class IntakeSubsystem implements Subsystem {
    /** Change this value to tune normal intake speed for the current robot. */
    public static final double INTAKE_POWER = 1.0;
    /** Change this value to tune reverse eject speed for the current robot. */
    public static final double EJECT_POWER = 1.0;

    private enum RequestedMode {
        IDLE,
        INTAKING,
        HOLDING,
        EJECTING
    }

    private final IntakeHardware intakeHardware;
    private final IdleIntakeState idleState;
    private final IntakingState intakingState;
    private final HoldingState holdingState;
    private final EjectingState ejectingState;
    private final FSM fsm;

    private RequestedMode requestedMode = RequestedMode.IDLE;

    /** Creates an intake subsystem that sends all motor commands through {@code intakeHardware}. */
    public IntakeSubsystem(IntakeHardware intakeHardware) {
        if (intakeHardware == null) {
            throw new IllegalArgumentException("Intake subsystem needs intake hardware.");
        }

        this.intakeHardware = intakeHardware;
        idleState = new IdleIntakeState(this);
        intakingState = new IntakingState(this);
        holdingState = new HoldingState(this);
        ejectingState = new EjectingState(this);
        fsm = new FSM(idleState);

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
        requestedMode = RequestedMode.IDLE;
        intakeHardware.stop();
    }

    @Override
    public String getName() {
        return "Intake";
    }

    /** Requests normal forward intake behavior. */
    public void startIntake() {
        requestedMode = RequestedMode.INTAKING;
    }

    /** Requests the safe stopped intake behavior. */
    public void stopIntake() {
        requestedMode = RequestedMode.IDLE;
    }

    /** Requests baseline holding behavior. */
    public void hold() {
        requestedMode = RequestedMode.HOLDING;
    }

    /** Requests reverse eject behavior. */
    public void eject() {
        requestedMode = RequestedMode.EJECTING;
    }

    /** Returns the current state name for telemetry. */
    public String getCurrentStateName() {
        String currentStateName = fsm.getCurrentStateName();
        return currentStateName == null ? idleState.getName() : currentStateName;
    }

    /** Returns whether the optional intake motor is available. */
    public boolean isAvailable() {
        return intakeHardware.isAvailable();
    }

    void runIntakeForward() {
        intakeHardware.forward(INTAKE_POWER);
    }

    void runIntakeReverse() {
        intakeHardware.reverse(EJECT_POWER);
    }

    void stopOutput() {
        intakeHardware.stop();
    }

    private void addTransitions() {
        fsm.addTransition(new Transition(idleState, intakingState,
                () -> requestedMode == RequestedMode.INTAKING));
        fsm.addTransition(new Transition(idleState, holdingState,
                () -> requestedMode == RequestedMode.HOLDING));
        fsm.addTransition(new Transition(idleState, ejectingState,
                () -> requestedMode == RequestedMode.EJECTING));
        fsm.addTransition(new Transition(intakingState, idleState,
                () -> requestedMode == RequestedMode.IDLE));
        fsm.addTransition(new Transition(intakingState, holdingState,
                () -> requestedMode == RequestedMode.HOLDING));
        fsm.addTransition(new Transition(intakingState, ejectingState,
                () -> requestedMode == RequestedMode.EJECTING));
        fsm.addTransition(new Transition(holdingState, idleState,
                () -> requestedMode == RequestedMode.IDLE));
        fsm.addTransition(new Transition(holdingState, intakingState,
                () -> requestedMode == RequestedMode.INTAKING));
        fsm.addTransition(new Transition(holdingState, ejectingState,
                () -> requestedMode == RequestedMode.EJECTING));
        fsm.addTransition(new Transition(ejectingState, idleState,
                () -> requestedMode == RequestedMode.IDLE));
        fsm.addTransition(new Transition(ejectingState, intakingState,
                () -> requestedMode == RequestedMode.INTAKING));
        fsm.addTransition(new Transition(ejectingState, holdingState,
                () -> requestedMode == RequestedMode.HOLDING));
    }
}
