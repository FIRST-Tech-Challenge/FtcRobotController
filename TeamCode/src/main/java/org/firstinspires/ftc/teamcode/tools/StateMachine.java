package org.firstinspires.ftc.teamcode.tools;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;

// StateMachine class that handles state management and transitions
public class StateMachine {

    // Inner State class representing an individual state within the StateMachine
    public static class State {
        private final ArrayList<Transition> transitions = new ArrayList<>(); // List of transitions from this state
        private Transition currentTransition = null; // The transition that is currently being processed
        public String name; // Name identifier for the state

        // Constructor that sets the name of the state
        public State(String name) {
            this.name = name;
        }

        // Method to add a transition to this state
        public void addTransitionTo(State nextState, BooleanSupplier trigger, Actions actions)
        {
            transitions.add(new Transition(nextState, trigger, actions));
        }

        // Inner Transition class representing a possible change from the current state to another
        public class Transition {
            private final BooleanSupplier trigger; // The condition under which this transition will occur
            private final State nextState; // The state to transit=ion to
            //ArrayList<Action> actions; // List of actions to be performed during the transition
            private Actions actions;

            // Constructor that initializes a transition
            public Transition(State nextState, BooleanSupplier trigger, Actions actions) {
                this.nextState = nextState;
                this.trigger = trigger;
                this.actions = actions;
            }

            // Method to determine if this transition's condition is met
            public boolean triggered() {
                return trigger.getAsBoolean();
            }


            // Method to determine if all actions for this transition are complete
            public boolean isTransitionComplete() {
                return actions.areActionsComplete();
            }

            // Method to get the state that this transition leads to
            public State getDestinationState() {
                return nextState;
            }

        }

        public State updateSync() {
            State latestState = update(); //Do one call to process potential trigger
            while(currentTransition!=null){ // Let's wait until transitions are fully executed
                latestState = update();
            }
            return latestState;
        }
        // Method called every update cycle to determine if the state should transition
        State update() {
            // If there is no active transition, check for a triggered transition
            if (currentTransition == null) {
                for (Transition t : transitions) {
                    if (t.triggered()) {
                        currentTransition = t;
                        break;
                    }
                }
                if(currentTransition == null) {
                    return this; // Still no transition triggered, stay in this state
                }
            }

            // If the current transition is complete, move to the next state
            if (currentTransition.isTransitionComplete()) {
                State destinationState = currentTransition.getDestinationState();
                currentTransition = null;
                return destinationState; // Transition to the next state
            }
            return this; // Transition not complete, stay in this state
        }
    }

    private final ArrayList<State> states = new ArrayList<>(); // List of all states in the state machine
    public State currentState = null; // The state machine's current state

    // Method to add a new state to the state machine
    public void addState(State state) {
        states.add(state);
    }

    //TODO: Find the equivalent of an assert that gives an error message, and doesn't crash the code.

    // Method to set the initial state of the state machine. Can only be done once.
    public void setInitialState(State state) {
        assert currentState == null : "cannot set initial state twice"; // Ensure initial state is not already set
        currentState = state;
    }

    // Method to progress the state machine to the next state if a transition is triggered
    public void updateState() {
        assert currentState != null : "initial state undefined"; // Ensure that there is a current state to update from
        currentState = currentState.update(); // Update the current state, which may cause a state transition
    }

    public void updateStateSync() {
        assert currentState != null : "initial state undefined"; // Ensure that there is a current state to update from
        currentState = currentState.updateSync(); // Update the current state, which may cause a state transition
    }

    public State getCurrentState(){
        return currentState;
    }
}

