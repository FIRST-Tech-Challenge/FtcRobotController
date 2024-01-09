package org.firstinspires.ftc.teamcode.AutoCode.Trajectory;

import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.MecDrive;
import org.firstinspires.ftc.teamcode.drivebase.StateM.StateMMovmentPerformer;
import org.firstinspires.ftc.teamcode.drivebase.StateM.StateMachine;

import static org.firstinspires.ftc.teamcode.MecDrive.LHook;
import static org.firstinspires.ftc.teamcode.MecDrive.RHook;
import static org.firstinspires.ftc.teamcode.MecDrive.airplane;
import static org.firstinspires.ftc.teamcode.drivebase.StateM.StateMachine.ReturnState.PROCEED;

public class SpikeFinish extends StateMachine<SpikeFinish.State> implements StateMMovmentPerformer {

    public double angle;


    public enum State {
        START,
        VERIABLE,
        IDLE,


    }
    // HANDS UP = start loosening hang moters
    // DON'T A SHOOT = move servo
    // STOP A MOVING = Stop hang moters
    // BANG = airplane FIRES
    // ALL THE WAY = Start loosening hang moters AGAIN :(
    // I SAID ALL THE WAY = Move servo AGAIN :(
    // STOP OR SHOT = stop hang moters AGAIN :(


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
    public SpikeFinish() {
        state = State.START;

    }
    @Override
    public ReturnState runIteration() {
        switch (state) {

            case START: {

                if (getElapsedStateTime() > 1) {
                    switchState(State.VERIABLE);
                }
                break;
            }
            case VERIABLE: {
                if(getElapsedStateTime() > 10) {
                    Globals.SpikeIsFinished = true;
                    switchState(State.IDLE);
                }
                break;
            }
            case IDLE: {
                if(getElapsedStateTime() > 1) {
                    MecDrive.RESETME = true;
                    return PROCEED;
                }
                break;
            }

        }
        return ReturnState.KEEP_RUNNING_ME;
    }
}
