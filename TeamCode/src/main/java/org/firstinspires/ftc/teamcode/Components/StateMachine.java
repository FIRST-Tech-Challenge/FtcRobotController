package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Components.StateMachine.States.DROPPED;
import static org.firstinspires.ftc.teamcode.Components.StateMachine.States.INTAKE_DOWN;
import static org.firstinspires.ftc.teamcode.Components.StateMachine.States.IN_WAREHOUSE;
import static org.firstinspires.ftc.teamcode.Components.StateMachine.States.TURRET_STRAIGHT;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class StateMachine {
    boolean teleOp = false;
    public enum States {
        INTAKING(false),
        FLIPPING(false),
        SWITCHED(false),
        TRANSFERRING(false),
        INTAKE_DOWN(false),
        BASKET_TRANSFER(true),
        BASKET_CIELING(false),
        BASKET_DROP(false),
        BASKET_ARM_REST(true),
        SLIDING(false),
        EXTENDED(false),
        RAISED(false),
        TURRET_STRAIGHT(true),
        IN_WAREHOUSE(false),
        TRANSFERRED(false),
        DROPPED(false);
        boolean status;
        States(boolean value) {
            this.status = value;
        }

        public void setStatus(boolean status) {
            this.status = status;
        }
    }

    public StateMachine(LinearOpMode op, boolean isTeleOp) {
        States.values();
        if (isTeleOp) {
            INTAKE_DOWN.setStatus(false);
        }
        teleOp=isTeleOp;
    }

    public void setState(States state, boolean value) {
        state.setStatus(value);
    }

    public boolean checkIf(States state){
        if(state == States.INTAKING){
            return teleOp || IN_WAREHOUSE.status;
        }
        else if(state == States.FLIPPING){
            return States.SWITCHED.status && TURRET_STRAIGHT.status && !States.EXTENDED.status && !States.RAISED.status && States.BASKET_TRANSFER.status;
        }
        else if(state == States.SWITCHED){
            return true;
        }
        else if(state == States.TRANSFERRING){
            return States.SWITCHED.status && TURRET_STRAIGHT.status && !States.EXTENDED.status && !States.RAISED.status && States.BASKET_TRANSFER.status && !States.INTAKE_DOWN.status;
        }
        else if(state == States.INTAKE_DOWN){
            return !States.SWITCHED.status;
        }
        else if(state == States.BASKET_CIELING){
            return States.BASKET_DROP.status && DROPPED.status  || States.BASKET_TRANSFER.status && States.TRANSFERRED.status;
        }
        else if(state == States.BASKET_ARM_REST){

        }
        else if(state == States.BASKET_DROP){

        }
        else if(state == States.BASKET_TRANSFER){

        }
        else if(state == States.EXTENDED){

        }
        else if(state == States.IN_WAREHOUSE){

        }
        else if(state == States.RAISED){

        }
        else if(state == States.SLIDING){

        }
        else{
            return false;
        }
        return false;
    }
}
