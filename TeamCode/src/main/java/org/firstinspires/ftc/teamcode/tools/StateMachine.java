package org.firstinspires.ftc.teamcode.tools;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;



// StateMachine class that handles state management and transitions
public class StateMachine {
    Telemetry telemetry;

    // Inner State class representing an individual state within the StateMachine
    public class State {
        private final ArrayList<Transition> transitions = new ArrayList<>(); // List of transitions from this state
        private Transition currentTransition = null; // The transition that is currently being processed
        public String name; // Name identifier for the state

        // Constructor that sets the name of the state
        public State(String name) {
            this.name = name;
        }

        // Method to add a transition to this state
        public void addTransition(Transition transition) {
            transitions.add(transition);
        }

        // Inner Transition class representing a possible change from the current state to another
        public class Transition {
            private final BooleanSupplier trigger; // The condition under which this transition will occur
            private final State nextState; // The state to transition to
            ArrayList<Action> actions; // List of actions to be performed during the transition
            int currentActionIndex; // Index to keep track of the current action

            // Constructor that initializes a transition
            public Transition(State nextState, BooleanSupplier trigger, ArrayList<Action> actions) {
                this.nextState = nextState;
                this.trigger = trigger;
                this.actions = actions;
            }

            // Inner Action class representing a single actionable item within a transition
            public class Action {
                private final BooleanSupplier performAction; // A function that performs the action and returns true if the action is complete

                // Constructor that sets the action
                public Action(BooleanSupplier performAction) {
                    this.performAction = performAction;
                }

                // Method to determine if the action is complete based on the BooleanSupplier
                public boolean actionComplete() {
                    return performAction.getAsBoolean();
                }
            }

            // Method to determine if this transition's condition is met
            public boolean triggered() {
                return trigger.getAsBoolean();
            }

            // Method to determine if all actions for this transition are complete
            public boolean transitionComplete() {
                // Iterate through all actions to see if they are complete
                for (currentActionIndex = 0; currentActionIndex < actions.size(); currentActionIndex++) {
                    if (!(actions.get(currentActionIndex).actionComplete())) {
                        return false;
                    }
                }
                currentActionIndex = 0; // Reset the action index
                return true; // All actions are complete
            }

            // Method to get the state that this transition leads to
            public State getDestinationState() {
                return nextState;
            }
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
                return this; // No transition triggered, stay in this state
            }
            // If the current transition is complete, move to the next state
            if (currentTransition.transitionComplete()) {
                State destinationState = currentTransition.getDestinationState();
                currentTransition = null;
                return destinationState; // Transition to the next state
            }
            return this; // Transition not complete, stay in this state
        }
    }

    private final ArrayList<State> states = new ArrayList<>(); // List of all states in the state machine
    private State currentState = null; // The state machine's current state

    // Method to add a new state to the state machine
    public void addState(State state) {
        states.add(state);
    }

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


    // method for a servo to correct itself if it is not within one degree of the desired position
    public void setPosServo(Servo servo, int pos) {
        int tolerance = 1;
        servo.setPosition(pos);
        int counter = 0; // counts the number of times the correctional loop runs
        // is my error greater or less than error
        while (Math.abs(pos - servo.getPosition()) >= tolerance && counter <= 5) {
            telemetry.addLine(servo + " correction started");
            // divide by two on the correctional value so that it does not overshoot by too much
            if (pos - servo.getPosition() > 0) {servo.setPosition(pos + (pos - servo.getPosition()) / 2);}
            if (pos - servo.getPosition() < 0) {servo.setPosition(pos + (servo.getPosition() - pos) / 2);}
            counter++;
            telemetry.addLine(servo + " corrected " + counter + " times");
        }
        telemetry.addLine(servo + " correction finished");
    }
}
