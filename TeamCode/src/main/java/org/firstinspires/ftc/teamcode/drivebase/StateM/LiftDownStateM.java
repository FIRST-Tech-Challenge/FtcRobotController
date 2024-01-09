package org.firstinspires.ftc.teamcode.drivebase.StateM;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.MecDrive;

import static org.firstinspires.ftc.teamcode.drivebase.StateM.StateMachine.ReturnState.PROCEED;

public class LiftDownStateM extends StateMachine<LiftDownStateM.State> implements StateMMovmentPerformer {

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
    public LiftDownStateM() {
        state = State.START;

    }
    @Override
    public ReturnState runIteration() {
        switch (state) {

            case START: {
                if (MecDrive.Lift.getCurrentPosition() < 1000) {switchState(State.REVERSEPOSISTIONS);}
                else {switchState(State.UPFIRST);}
                break;
            }
            case UPFIRST: {
                MecDrive.Lift.setTargetPosition(-1000);
                MecDrive.Lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                MecDrive.Lift.setPower(1);
                if(getElapsedStateTime() > 500) {switchState(State.REVERSEPOSISTIONS);}
                break;
            }
            case REVERSEPOSISTIONS: {
                MecDrive.SLift.setPosition(.9);
                MecDrive.Pivot.setPosition(.7);
                MecDrive.Door.setPosition(.9);

                if(getElapsedStateTime() > 500) {
                    switchState(State.LOWERTHATLIFT);
                }
                break;
            }
            case LOWERTHATLIFT: {
                MecDrive.Lift.setTargetPosition(-1);
                MecDrive.Lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                MecDrive.Lift.setPower(1);

                if(getElapsedStateTime() > 1000) {
                    switchState(State.IDLE);
                }
                break;
            }

            case IDLE: {
                if(getElapsedStateTime() > 100) {
                    MecDrive.RESETME = true;
                    return PROCEED;
                }
                break;
            }

        }
        return ReturnState.KEEP_RUNNING_ME;
    }
}
