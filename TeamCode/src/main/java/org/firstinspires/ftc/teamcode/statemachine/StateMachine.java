package org.firstinspires.ftc.teamcode.statemachine;

import java.util.ArrayList;
import java.util.List;

public class StateMachine {

    private final Stage stage;
    private final StateSwitchAction stateSwitchAction;
    private final StateEndAction stateEndAction;
    private final List<State> states;

    public static Builder builder() {
        return new Builder();
    }

    public static class Builder {

        private Stage stage;
        private StateSwitchAction stateSwitchAction;
        private StateEndAction stateEndAction;
        private List<State> states;

        private Builder() {
            states = new ArrayList<>();
        }

        public Builder stateSwitchAction(final StateSwitchAction stateSwitchAction) {
            this.stateSwitchAction = stateSwitchAction;
            return this;
        }

        public Builder stateEndAction(final StateEndAction stateEndAction) {
            this.stateEndAction = stateEndAction;
            return this;
        }

        public Builder stage(final Stage stage) {
            this.stage = stage;
            return this;
        }

        public Builder addState(final State state) {
            states.add(state);
            return this;
        }

        public Builder addSingleState(final SingleState singleState) {
            states.add(() -> {singleState.runState(); return true;});
            return this;
        }

        public Builder addMineralState(final MineralStateProvider mineralStateProvider, State left, State middle, State right) {
            states.add(() -> {
                switch (mineralStateProvider.getMineralState()) {
                    case 0:
                        return left.runState();
                    case 1:
                        return middle.runState();
                    case 2:
                        return right.runState();
                    default:
                        return true;
                }
            });
            return this;
        }

        public Builder addTimedState(final float seconds, final SingleState start, final SingleState stop) {
            states.add(new State() {
                private float seconds;
                private long timer;
                private boolean started = false;

                @Override
                public boolean runState() {
                    if (!started) {
                        timer = futureTime(seconds);
                        start.runState();
                        started = true;
                        return false;
                    } else {
                        if (System.nanoTime() >= timer) {
                            stop.runState();
                            started = false;
                            return true;
                        } else {
                            return false;
                        }
                    }
                }

                public State setTime(float seconds) {
                    this.seconds = seconds;
                    return this;
                }

            }.setTime(seconds));
            return this;
        }
        
        public Builder addSimultaneousStates(final State... newStates) {
            states.add(new State() {
                private State[] newStates;
                private boolean[] completed;
                private int statesRemaining;
                
                @Override
                public boolean runState() {
                    for (int i = 0; i < newStates.length; i++)
                        if (!completed[i] && newStates[i].runState()) {
                            completed[i] = true;
                            statesRemaining--;
                        }
                    if (statesRemaining == 0) {
                        completed = new boolean[newStates.length];
                        statesRemaining = newStates.length;
                        return true;
                    } else
                        return false;
                }
                
                public State init(State[] newStates) {
                    this.newStates = newStates;
                    completed = new boolean[newStates.length];
                    statesRemaining = newStates.length;
                    return this;
                }
                
            }.init(newStates));
            return this;
        }

        private static long futureTime(float seconds){ return System.nanoTime() + (long) (seconds * 1e9); }

        public Builder addNestedStateMachine(final StateMachine stateMachine) {
            states.add(() -> stateMachine.execute());
            return this;
        }

        public StateMachine build() {
            if (stage == null)
                throw new NullPointerException("stage can not be null");
            if (stateSwitchAction == null)
                throw new NullPointerException("stateSwitchAction can not be null");
            if (stateEndAction == null)
                throw new NullPointerException("stateEndAction can not be null");
            return new StateMachine(stage, stateSwitchAction, stateEndAction, states);
        }

    }

    private StateMachine(final Stage stage, final StateSwitchAction stateSwitchAction, final StateEndAction stateEndAction, final List<State> states) {
        this.stage = stage;
        this.stateSwitchAction = stateSwitchAction;
        this.stateEndAction = stateEndAction;
        this.states = states;
    }

    public boolean execute() {
        if (stage.getStage() == states.size()) {
            stateEndAction.onStateEnd();
            stage.resetStage();
            return true;
        } else {
            State state = states.get(stage.getStage());
            if (state.runState()) {
                stateSwitchAction.onStateSwitch();
                stage.incrementStage();
            }
            return false;
        }
    }

}