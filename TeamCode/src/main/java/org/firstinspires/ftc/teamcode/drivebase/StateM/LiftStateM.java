package org.firstinspires.ftc.teamcode.drivebase.StateM;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.MecDrive;

import static org.firstinspires.ftc.teamcode.drivebase.StateM.StateMachine.ReturnState.PROCEED;

public class LiftStateM extends StateMachine<LiftStateM.State> implements StateMMovmentPerformer {

    public double angle;


    public enum State {
        START,
        PIVOTWITHTHATDOOR,
        RAISETHATLIFT,
        POSISTIONS,
        IDLE,


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
    public LiftStateM() {
        state = State.START;

    }
    @Override
    public ReturnState runIteration() {
        switch (state) {

            case START: {

                if (getElapsedStateTime() > 500) {
                    switchState(State.PIVOTWITHTHATDOOR);
                }
                break;
            }
            case PIVOTWITHTHATDOOR: {
                MecDrive.SLift.setPosition(.15);
                MecDrive.Pivot.setPosition(.8);
                MecDrive.Door.setPosition(1);

                if(getElapsedStateTime() >500) {
                    switchState(State.RAISETHATLIFT);
                }
                break;
            }
            case RAISETHATLIFT: {
                MecDrive.Lift.setTargetPosition(-1400);
                MecDrive.Lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                MecDrive.Lift.setPower(1);

                if(getElapsedStateTime() > 200) {

                    switchState(State.POSISTIONS);
                }
                break;
            }
            case POSISTIONS: {
                MecDrive.SLift.setPosition(.3);
                MecDrive.Pivot.setPosition(1);

                if(getElapsedStateTime() > 200) {

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
