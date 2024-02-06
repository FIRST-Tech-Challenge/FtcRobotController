package org.firstinspires.ftc.teamcode.drivebase.StateM;

import org.firstinspires.ftc.teamcode.MecDrive;

import static org.firstinspires.ftc.teamcode.MecDrive.LHook;
import static org.firstinspires.ftc.teamcode.MecDrive.RHook;
import static org.firstinspires.ftc.teamcode.MecDrive.airplane;
import static org.firstinspires.ftc.teamcode.drivebase.StateM.StateMachine.ReturnState.PROCEED;

public class HangStateM extends StateMachine<HangStateM.State> implements StateMMovmentPerformer {

    public double angle;


    public enum State {
        START,
        HANDSUP,
        DONTASHOOT,
        STOPAMOVING,
//        BANG,
//        BEHINDTHEBACK,
//        THEREUGO,
//        ALLTHEWAY,
//        ISAIDALLTHEWAY,
//        STOPORSHOT,
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
    public HangStateM() {
        state = State.START;

    }
    @Override
    public ReturnState runIteration() {
        switch (state) {

            case START: {

                if (getElapsedStateTime() > 1) {
                    switchState(State.HANDSUP);
                }
                break;
            }
            case HANDSUP: {
                MecDrive.LHang.setPower(.3);
                MecDrive.RHang.setPower(.3);
                if(getElapsedStateTime() > 1000) {
                    switchState(State.DONTASHOOT);
                }
                break;
            }
            case DONTASHOOT: {
                RHook.setPosition(0);
                LHook.setPosition(1);
                if(getElapsedStateTime() > 1000) {
                    switchState(State.STOPAMOVING);
                }
                break;
            }
            case STOPAMOVING: {
                MecDrive.LHang.setPower(0);
                MecDrive.RHang.setPower(0);
                if(getElapsedStateTime() > 750) {

                    switchState(State.IDLE);
                }
                break;
            }
//            case BANG: {
//                airplane.setPosition(0);
//                if(getElapsedStateTime() > 1000) {
//
//                    switchState(State.THEREUGO);
//                }
//                break;
//            }

//            case BEHINDTHEBACK: {
//                MecDrive.LHang.setPower(-.2);
//                MecDrive.RHang.setPower(-.2);
//                if(getElapsedStateTime() > 500) {
//
//                    switchState(State.THEREUGO);
//                }
//                break;
//            }

//            case THEREUGO: {
////                airplane.setPosition(.3);
////                if(getElapsedStateTime() > 500) {
//                RHook.setPosition(1);
//                LHook.setPosition(0);
//                MecDrive.LHang.setPower(0);
//                MecDrive.RHang.setPower(0);
////                }
//                if(getElapsedStateTime() > 500) {
//
//                    switchState(State.ALLTHEWAY);
//                }
//                break;
//            }
//            case ALLTHEWAY: {
//                MecDrive.LHang.setPower(.3);
//                MecDrive.RHang.setPower(.3);
//                if(getElapsedStateTime() > 1000) {
//
//                    switchState(State.ISAIDALLTHEWAY);
//                }
//                break;
//            }
//            case ISAIDALLTHEWAY: {
//                RHook.setPosition(0);
//                LHook.setPosition(1);
//                if(getElapsedStateTime() > 500) {
//
//                    switchState(State.STOPORSHOT);
//                }
//                break;
//            }
//            case STOPORSHOT: {
//                MecDrive.LHang.setPower(0);
//                MecDrive.RHang.setPower(0);
//                if(getElapsedStateTime() > 250) {
//
//                    switchState(State.IDLE);
//                }
//                break;
//            }

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
