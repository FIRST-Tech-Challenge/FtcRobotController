package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;

/**
 * A hashmap-backed finite state machine, where each state is an FTCLib {@link Command}
 * and transitions between states are conditions polled once per scheduler loop.
 *
 * <p>{@code StateMachine} itself is a {@link Command}, so it's scheduled like any other:
 * once running, it drives whichever state's {@code Command} is active and switches states
 * when a transition's condition becomes true.
 *
 * <p>Configuration is done through {@link #builder()}, which always configures the
 * singleton returned by {@link #getInstance()}. Call {@link #resetInstance()} at the
 * start of an opmode's {@code init()} before rebuilding, otherwise old states and
 * transitions from a previous run will still be attached.
 *
 * <pre>{@code
 * enum State { IDLE, INTAKE, TRANSFER, SCORE }
 *
 * StateMachine.resetInstance();
 *
 * StateMachine sm = StateMachine.builder()
 *     .state(State.IDLE, new IdleCommand())
 *     .state(State.INTAKE, new IntakeCommand())
 *     .state(State.TRANSFER, new TransferCommand())
 *     .state(State.SCORE, new ScoreCommand())
 *     .initial(State.IDLE)
 *     .transition(State.IDLE, State.INTAKE, () -> gamepad1.a)
 *     .transition(State.INTAKE, State.TRANSFER, () -> intakeSensor.hasGamePiece())
 *     .transition(State.TRANSFER, State.SCORE, () -> gamepad1.b)
 *     .anyTransition(State.IDLE, () -> gamepad1.back)
 *     .onEnter(State.INTAKE, () -> intakeMotor.setPower(1.0))
 *     .onExit(State.INTAKE, () -> intakeMotor.setPower(0.0))
 *     .telemetry(telemetry)
 *     .onStateChange((from, to) -> RobotLog.dd("SM", from + " -> " + to))
 *     .build();
 *
 * schedule(sm);
 * }</pre>
 */
public class StateMachine extends CommandBase {
    private static StateMachine instance;

    private final Map<Object, Command> states = new HashMap<>();
    private final Map<Object, Runnable> onEnterHooks = new HashMap<>();
    private final Map<Object, Runnable> onExitHooks = new HashMap<>();
    private final Map<Object, List<Transition>> transitions = new HashMap<>();
    private final List<Transition> globalTransitions = new ArrayList<>();

    private Object initialKey;
    private Object currentKey;
    private Command currentCommand;

    private Telemetry telemetry;
    private BiConsumer<Object, Object> stateChangeListener;

    private StateMachine() {}

    /**
     * Returns the shared {@code StateMachine} instance, creating it on first call.
     */
    public static StateMachine getInstance() {
        if (instance == null) {
            instance = new StateMachine();
        }
        return instance;
    }

    /**
     * Discards the current singleton so the next {@link #getInstance()} or
     * {@link #builder()} call starts from a clean slate. Call this at the top of an
     * opmode's {@code init()} — without it, states/transitions from a previous opmode
     * run in the same app session remain registered.
     */
    public static void resetInstance() {
        instance = null;
    }

    private static class Transition {
        final Object to;
        final BooleanSupplier condition;
        Transition(Object to, BooleanSupplier condition) {
            this.to = to;
            this.condition = condition;
        }
    }

    /**
     * Starts a {@link Builder} bound to the current singleton (see {@link #getInstance()}).
     */
    public static Builder builder() {
        return new Builder(getInstance());
    }

    /**
     * Fluent configuration surface for {@link StateMachine}. All methods mutate the
     * bound singleton and return {@code this}, so calls can be chained. Every key
     * used with {@link #state}, {@link #transition}, {@link #onEnter}, etc. should be
     * a stable, distinct identifier — an {@code enum} is strongly recommended over
     * raw strings or ints to avoid typos causing silent no-op transitions.
     */
    public static class Builder {
        private final StateMachine sm;

        private Builder(StateMachine sm) {
            this.sm = sm;
        }

        /**
         * Registers a state and the {@link Command} that runs while it's active.
         *
         * @param key     identifier for this state, e.g. an enum constant
         * @param command command whose {@code initialize()}/{@code execute()}/{@code end()}
         *                run while this state is current
         */
        public Builder state(Object key, Command command) {
            sm.states.put(key, command);
            return this;
        }

        /**
         * Sets which registered state the machine starts in when scheduled.
         */
        public Builder initial(Object key) {
            sm.initialKey = key;
            return this;
        }

        /**
         * Adds a transition that's only checked while the machine is in {@code from}.
         * Conditions are polled every {@code execute()}; the first matching transition
         * (in registration order) wins.
         *
         * <pre>{@code
         * .transition(State.INTAKE, State.TRANSFER, () -> intakeSensor.hasGamePiece())
         * }</pre>
         *
         * @param from      state this transition applies to
         * @param to        state to switch to when {@code condition} is true
         * @param condition polled once per loop while current state is {@code from}
         */
        public Builder transition(Object from, Object to, BooleanSupplier condition) {
            sm.transitions
                    .computeIfAbsent(from, k -> new ArrayList<>())
                    .add(new Transition(to, condition));
            return this;
        }

        /**
         * Adds a transition checked from every state, evaluated before any
         * state-specific transition. Useful for a global abort/reset button:
         *
         * <pre>{@code
         * .anyTransition(State.IDLE, () -> gamepad1.back)
         * }</pre>
         */
        public Builder anyTransition(Object to, BooleanSupplier condition) {
            sm.globalTransitions.add(new Transition(to, condition));
            return this;
        }

        /**
         * Registers a hook that runs once when the machine switches into {@code key},
         * before that state's {@code Command.initialize()}. Distinct from the
         * command's own lifecycle so the {@link Command} stays reusable outside the
         * state machine — use this for state-machine-specific setup like zeroing a
         * timer or spinning up an intake:
         *
         * <pre>{@code
         * .onEnter(State.INTAKE, () -> intakeMotor.setPower(1.0))
         * }</pre>
         */
        public Builder onEnter(Object key, Runnable hook) {
            sm.onEnterHooks.put(key, hook);
            return this;
        }

        /**
         * Registers a hook that runs once when the machine switches out of {@code key},
         * after that state's {@code Command.end()}.
         *
         * <pre>{@code
         * .onExit(State.INTAKE, () -> intakeMotor.setPower(0.0))
         * }</pre>
         */
        public Builder onExit(Object key, Runnable hook) {
            sm.onExitHooks.put(key, hook);
            return this;
        }

        /**
         * Enables automatic per-loop telemetry of the current state under the key
         * {@code "StateMachine/currentState"}. Does not call {@code telemetry.update()}
         * — most opmodes already call that once per loop, and calling it again here
         * would double-flush and can hide other telemetry lines added elsewhere.
         */
        public Builder telemetry(Telemetry telemetry) {
            sm.telemetry = telemetry;
            return this;
        }

        /**
         * Registers a listener invoked on every state change with
         * {@code (previousKey, newKey)}. {@code previousKey} is {@code null} for the
         * very first entry into the initial state. Useful for logging to FTC Dashboard,
         * {@code RobotLog}, or a custom overlay:
         *
         * <pre>{@code
         * .onStateChange((from, to) -> RobotLog.dd("SM", from + " -> " + to))
         * }</pre>
         */
        public Builder onStateChange(BiConsumer<Object, Object> listener) {
            sm.stateChangeListener = listener;
            return this;
        }

        /**
         * Validates the configuration and returns the configured {@link StateMachine},
         * ready to be scheduled.
         *
         * @throws IllegalStateException if {@link #initial} was never called, or was
         *                                given a key never passed to {@link #state}
         */
        public StateMachine build() {
            if (sm.initialKey == null) {
                throw new IllegalStateException("StateMachine: no initial state set");
            }
            if (!sm.states.containsKey(sm.initialKey)) {
                throw new IllegalStateException("StateMachine: initial state was never registered with .state()");
            }
            return sm;
        }
    }

    @Override
    public void initialize() {
        currentKey = initialKey;
        currentCommand = states.get(currentKey);
        runEnterHook(currentKey);
        if (currentCommand != null) {
            currentCommand.initialize();
        }
        notifyStateChange(null, currentKey);
    }

    @Override
    public void execute() {
        if (currentCommand == null) {
            reportTelemetry();
            return;
        }

        Object nextKey = checkTransitions();
        if (nextKey != null) {
            switchState(nextKey);
            reportTelemetry();
            return;
        }

        currentCommand.execute();
        if (currentCommand.isFinished()) {
            currentCommand.end(false);
        }

        reportTelemetry();
    }

    /**
     * @return the key of the first matching transition ({@link #globalTransitions}
     *         checked before state-local ones), or {@code null} if none match
     */
    private Object checkTransitions() {
        for (Transition t : globalTransitions) {
            if (t.condition.getAsBoolean()) return t.to;
        }
        List<Transition> local = transitions.get(currentKey);
        if (local != null) {
            for (Transition t : local) {
                if (t.condition.getAsBoolean()) return t.to;
            }
        }
        return null;
    }

    private void switchState(Object nextKey) {
        Object previousKey = currentKey;

        if (currentCommand != null) {
            currentCommand.end(false);
        }
        runExitHook(previousKey);

        currentKey = nextKey;
        currentCommand = states.get(nextKey);
        if (currentCommand == null) {
            throw new IllegalStateException("StateMachine: transitioned to unregistered state " + nextKey);
        }

        runEnterHook(currentKey);
        currentCommand.initialize();

        notifyStateChange(previousKey, currentKey);
    }

    private void runEnterHook(Object key) {
        Runnable hook = onEnterHooks.get(key);
        if (hook != null) hook.run();
    }

    private void runExitHook(Object key) {
        Runnable hook = onExitHooks.get(key);
        if (hook != null) hook.run();
    }

    private void notifyStateChange(Object from, Object to) {
        if (stateChangeListener != null) {
            stateChangeListener.accept(from, to);
        }
    }

    private void reportTelemetry() {
        if (telemetry != null) {
            telemetry.addData("StateMachine/currentState", currentKey);
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (currentCommand != null) {
            currentCommand.end(interrupted);
        }
        runExitHook(currentKey);
    }

    /**
     * Always {@code false} — a state machine represents ongoing robot behavior and
     * should keep running for the life of the opmode rather than being auto-removed
     * by the scheduler. If you need the machine to end on its own, add a terminal
     * state's transition condition that you check externally, or extend this class.
     */
    @Override
    public boolean isFinished() {
        return false;
    }

    /**
     * @return the key of the currently active state
     */
    public Object getCurrentState() {
        return currentKey;
    }
}