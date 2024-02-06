package org.firstinspires.ftc.teamcode.drivebase.StateM;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Globals;

import static org.firstinspires.ftc.teamcode.drivebase.StateM.StateMachine.ReturnState.PROCEED;

public class FAILSAFELiftDownStateM extends StateMachine<FAILSAFELiftDownStateM.State> implements StateMMovmentPerformer {

    public double angle;


    public enum State {
        START,
        REVERSEPOSISTIONS,
        LOWERTHATLIFT,
        IDLE,

        UPFIRST,

    }
  // PIVOT WITH THAT DOOR = pivet up and close the door
  // RAISE THAT LIFT = raise the lift
  // POSISTIONS = set it to placing position



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
    public FAILSAFELiftDownStateM() {
        state = State.START;

    }
    @Override
    public ReturnState runIteration() {
        switch (state) {

            case START: {
                switchState(State.REVERSEPOSISTIONS);
                break;
            }

            case REVERSEPOSISTIONS: {
                Globals.SLift.setPosition(.9);
                Globals.Pivot.setPosition(.7);


                if(getElapsedStateTime() > 500) {
                    switchState(State.LOWERTHATLIFT);
                }
                break;
            }
            case LOWERTHATLIFT: {
                Globals.Lift.setTargetPosition(1400);
                Globals.Lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                Globals.Lift.setPower(1);
                Globals.Door.setPosition(.9);

                if(getElapsedStateTime() > 1000) {
                    switchState(State.IDLE);
                }
                break;
            }

            case IDLE: {
                if(getElapsedStateTime() > 100) {
                    Globals.RESETME = true;
                    return PROCEED;
                }
                break;
            }

        }
        return ReturnState.KEEP_RUNNING_ME;
    }
}
