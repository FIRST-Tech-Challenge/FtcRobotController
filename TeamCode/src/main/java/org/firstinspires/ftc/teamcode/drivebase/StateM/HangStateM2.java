package org.firstinspires.ftc.teamcode.drivebase.StateM;

import org.firstinspires.ftc.teamcode.MecDrive;

import static org.firstinspires.ftc.teamcode.MecDrive.RHang;
import static org.firstinspires.ftc.teamcode.drivebase.StateM.StateMachine.ReturnState.PROCEED;

public class HangStateM2 extends StateMachine<HangStateM2.State> implements StateMMovmentPerformer {

    public double angle;


    public enum State {
        START,
        PULLUP,
        STOPAMOVING,
        LOCKYOURARMS,
        IDLE,


    }
    // PULL UP happens after the driver has lowered the bars with the hang and has gotten the hooks off
    // PULL UP = tightening the moters
    // STOP A MOVING = stop moving moters
    // LOCK YOUR ARMS = imiedietly after "STOP A MOVING" lower locks


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
    public HangStateM2() {
        state = State.START;

    }
    @Override
    public ReturnState runIteration() {
        switch (state) {

            case START: {

                if (getElapsedStateTime() > 1000) {
                    switchState(State.PULLUP);
                }
                break;
            }
            case PULLUP: {
                MecDrive.LHang.setPower(1);
                MecDrive.RHang.setPower(1);

                if(getElapsedStateTime() > 3000) {
                    switchState(State.STOPAMOVING);
                }
                break;
            }
            case STOPAMOVING: {
                MecDrive.LHang.setPower(0);
                MecDrive.RHang.setPower(0);

                if(getElapsedStateTime() > 100) {

                    switchState(State.LOCKYOURARMS);
                }
                break;
            }
            case LOCKYOURARMS: {
                MecDrive.LLock.setPosition(.25);
                MecDrive.RLock.setPosition(.75);

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
