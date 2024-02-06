package org.firstinspires.ftc.teamcode.AutoCode.Trajectory;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.MecDrive;
import org.firstinspires.ftc.teamcode.drivebase.StateM.StateMMovmentPerformer;
import org.firstinspires.ftc.teamcode.drivebase.StateM.StateMachine;

import static org.firstinspires.ftc.teamcode.drivebase.StateM.StateMachine.ReturnState.PROCEED;

public class LiftStateMAuto extends StateMachine<LiftStateMAuto.State> implements StateMMovmentPerformer {

    public double angle;


    public enum State {
        START,
        PIVOTWITHTHATDOOR,
        RAISETHATLIFT,
        POSISTIONS,
        LOWER,
        HIGHER,
        IDLE,


    }
  // PIVOT WITH THAT DOOR = pivet up and close the door
  // RAISE THAT LIFT = raise the lift
  // POSISTIONS = set it to placing position


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
    public LiftStateMAuto() {
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
                Globals.SLift.setPosition(.15);
                Globals.Pivot.setPosition(.8);
                Globals.Door.setPosition(1);

                if(getElapsedStateTime() > 500) {
                    switchState(State.RAISETHATLIFT);
                }
                break;
            }
            case RAISETHATLIFT: {
                Globals.Lift.setTargetPosition(-1200);
                Globals.Lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                Globals.Lift.setPower(1);

                if(getElapsedStateTime() > 200) {

                    switchState(State.POSISTIONS);
                }
                break;
            }
            case POSISTIONS: {
                Globals.SLift.setPosition(.3);
                Globals.Pivot.setPosition(1);

                if(getElapsedStateTime() > 1500) {

                    switchState(State.LOWER);
                }
                break;
            }
            case LOWER: {
                Globals.Lift.setTargetPosition(-867);
                Globals.Lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                Globals.Lift.setPower(1);


                if(getElapsedStateTime() > 3000) {
                    Globals.Door.setPosition(.9225);
                    switchState(State.HIGHER);
                }
                break;
            }
            case HIGHER: {
                    Globals.Lift.setTargetPosition(-1200);
                    Globals.Lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    Globals.Lift.setPower(1);

                if(getElapsedStateTime() > 2000) {

                    switchState(State.IDLE);
                }
                break;
            }

            case IDLE: {
                if(getElapsedStateTime() > 1000) {
                    Globals.WeHaveNoGoods = true;
//                    Globals.RESETME = true;
                    return PROCEED;
                }
                break;
            }

        }
        return ReturnState.KEEP_RUNNING_ME;
    }
}
