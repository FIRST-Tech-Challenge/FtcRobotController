package org.firstinspires.ftc.teamcode.AutoCode.Trajectory.StateM;

import org.firstinspires.ftc.teamcode.AutoCode.Trajectory.Base.StateMMovmentPerformer;
import org.firstinspires.ftc.teamcode.AutoCode.Trajectory.Base.StateMachine;

import static org.firstinspires.ftc.teamcode.AutoCode.Trajectory.Base.StateMachine.ReturnState.PROCEED;

public class AutoStateMBase extends StateMachine<AutoStateMBase.State> implements StateMMovmentPerformer {

    public double angle;


    public enum State {
        START,
        GOING,
        IDLE,


    }


    public State getStAte()
    {
        return state;
    }

    @Override
    public boolean run() {
        return runIteration() == PROCEED;
    }

    @Override
    public void reset() {
        state = State.START;

    }

    @Override
    public String getName() {
        return "AutoTransfer";
    }
    public AutoStateMBase() {
        state = State.START;

    }
    @Override
    public ReturnState runIteration() {
        switch (state) {

            case START: {

                if (getElapsedStateTime() > 1000) {
                    switchState(State.GOING);
                }
                break;
            }
            case GOING: {
                if (getElapsedStateTime() > 400) {
                    switchState(State.IDLE);
                }
                break;
            }
            case IDLE: {
                if(getElapsedStateTime() > 1000) {
                    return PROCEED;
                }
                break;
            }

        }
        return ReturnState.KEEP_RUNNING_ME;
    }
}
