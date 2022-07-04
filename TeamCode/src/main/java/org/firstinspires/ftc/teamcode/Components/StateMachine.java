package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Components.StateMachine.States.DROPPED;
import static org.firstinspires.ftc.teamcode.Components.StateMachine.States.FLIPPING;
import static org.firstinspires.ftc.teamcode.Components.StateMachine.States.INTAKING;
import static org.firstinspires.ftc.teamcode.Components.StateMachine.States.IN_WAREHOUSE;
import static org.firstinspires.ftc.teamcode.Components.StateMachine.States.SEQUENCING;
import static org.firstinspires.ftc.teamcode.Components.StateMachine.States.TURRET_STRAIGHT;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class StateMachine {
    boolean teleOp = false;
    Logger logger;

    public enum States {
        INTAKING(false, "INTAKING"),
        FLIPPING(false, "FLIPPING"),
        SWITCHED(false, "SWITCHED"),
        TRANSFERRING(false, "TRANSFERRING"),
        INTAKE_DOWN(true, "INTAKE_DOWN"),
        BASKET_TRANSFER(true, "BASKET_TRANSFER"),
        BASKET_CIELING(false, "BASKET_CIELING"),
        BASKET_DROP(false, "BASKET_DROP"),
        BASKET_ARM_REST(true, "BASKET_ARM_REST"),
        EXTENDED(false, "EXTENDED"),
        RAISED(false, "RAISED"),
        TURRET_STRAIGHT(true, "TURRET_STRAIGHT"),
        IN_WAREHOUSE(false, "IN_WAREHOUSE"),
        TRANSFERRED(false, "TRANSFERRED"),
        DROPPED(false, "DROPPED"),
        TURRET_SHORT(true, "TURRET_SHORT"),
        SEQUENCING(false, "SEQUENCING"),
        REVERSING(false, "REVERSING");
        boolean status;
        String name;

        States(boolean value, String name) {
            this.status = value;
        }

        public void setStatus(boolean status) {
            this.status = status;
        }
    }

    public StateMachine(LinearOpMode op, boolean isTeleOp, Logger log) {
        logger = log;
        teleOp = isTeleOp;
        if (teleOp) {
            States.INTAKE_DOWN.setStatus(true);
        }
    }

    public void setState(States state, boolean value) {
        if(state.status!=value) {
            state.setStatus(value);
            if (value == true) {
                logger.log("STATE:" + state.values()[1] + "true");
            } else {
                logger.log("STATE:" + state.values()[1] + "false");
            }
        }
    }

    public boolean getState(States state) {
        return state.status;
    }

    public boolean checkIf(States state) {
        if (state == States.INTAKING) {
            return (teleOp || IN_WAREHOUSE.status) && !INTAKING.status;
        } else if (state == States.REVERSING) {
            return (!FLIPPING.status && !States.REVERSING.status);
        } else if (state == SEQUENCING) {
            return States.SWITCHED.status;
        } else if (state == States.FLIPPING) {
            return TURRET_STRAIGHT.status && !States.EXTENDED.status && !States.RAISED.status && States.BASKET_TRANSFER.status && !FLIPPING.status;
        } else if (state == States.SWITCHED) {
            return true;
        } else if (state == States.TRANSFERRING) {
            return !States.TRANSFERRING.status && !FLIPPING.status && TURRET_STRAIGHT.status && !States.EXTENDED.status && !States.RAISED.status && States.BASKET_TRANSFER.status && !States.INTAKE_DOWN.status && SEQUENCING.status;
        } else if (state == States.INTAKE_DOWN) {
            return !States.SWITCHED.status;
        } else if (state == States.BASKET_CIELING) {
            return States.BASKET_DROP.status && DROPPED.status || States.BASKET_TRANSFER.status && States.TRANSFERRED.status;
        } else if (state == States.BASKET_ARM_REST) {
            return States.DROPPED.status;
        } else if (state == States.BASKET_DROP) {
            return !States.BASKET_ARM_REST.status && States.TRANSFERRED.status;
        } else if (state == States.BASKET_TRANSFER) {
            return States.BASKET_ARM_REST.status && !States.EXTENDED.status;
        } else if (state == States.EXTENDED) {
            return !States.BASKET_TRANSFER.status || !States.BASKET_ARM_REST.status;
        } else if (state == States.RAISED) {
            return States.TRANSFERRED.status;
        } else {
            return true;
        }
    }
}
