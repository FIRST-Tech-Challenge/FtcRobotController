package org.firstinspires.ftc.teamcode.Components;


import static org.firstinspires.ftc.teamcode.Components.StateMachine.BasketBasketArmStates.BASKET_ARM_ALLIANCE;
import static org.firstinspires.ftc.teamcode.Components.StateMachine.BasketBasketArmStates.BASKET_ARM_REST;
import static org.firstinspires.ftc.teamcode.Components.StateMachine.BasketBasketArmStates.BASKET_ARM_SHARED;
import static org.firstinspires.ftc.teamcode.Components.StateMachine.BasketBasketArmStates.BASKET_CEILING;
import static org.firstinspires.ftc.teamcode.Components.StateMachine.BasketBasketArmStates.BASKET_DROP;
import static org.firstinspires.ftc.teamcode.Components.StateMachine.BasketBasketArmStates.BASKET_TRANSFER;
import static org.firstinspires.ftc.teamcode.Components.StateMachine.BasketBasketArmStates.DROPPED;
import static org.firstinspires.ftc.teamcode.Components.StateMachine.IntakeStates.INTAKE_DOWN;
import static org.firstinspires.ftc.teamcode.Components.StateMachine.IntakeStates.INTAKE_FLIPPING_DOWN;
import static org.firstinspires.ftc.teamcode.Components.StateMachine.IntakeStates.INTAKE_FLIPPING_UP;
import static org.firstinspires.ftc.teamcode.Components.StateMachine.IntakeStates.INTAKE_REVERSING;
import static org.firstinspires.ftc.teamcode.Components.StateMachine.IntakeStates.INTAKE_SWITCHED;
import static org.firstinspires.ftc.teamcode.Components.StateMachine.IntakeStates.INTAKE_TRANSFERRED;
import static org.firstinspires.ftc.teamcode.Components.StateMachine.IntakeStates.INTAKE_TRANSFERRING;
import static org.firstinspires.ftc.teamcode.Components.StateMachine.IntakeStates.INTAKE_UP;
import static org.firstinspires.ftc.teamcode.Components.StateMachine.IntakeStates.INTAKING;
import static org.firstinspires.ftc.teamcode.Components.StateMachine.RobotStates.IN_WAREHOUSE;
import static org.firstinspires.ftc.teamcode.Components.StateMachine.RobotStates.SEQUENCING;
import static org.firstinspires.ftc.teamcode.Components.StateMachine.TurretStates.SLIDES_EXTENDED;
import static org.firstinspires.ftc.teamcode.Components.StateMachine.TurretStates.SLIDES_EXTENDING;
import static org.firstinspires.ftc.teamcode.Components.StateMachine.TurretStates.SLIDES_RETRACTED;
import static org.firstinspires.ftc.teamcode.Components.StateMachine.TurretStates.SLIDES_RETRACTING;
import static org.firstinspires.ftc.teamcode.Components.StateMachine.TurretStates.TURRET_FLAT;
import static org.firstinspires.ftc.teamcode.Components.StateMachine.TurretStates.TURRET_LOWERING;
import static org.firstinspires.ftc.teamcode.Components.StateMachine.TurretStates.TURRET_RAISED;
import static org.firstinspires.ftc.teamcode.Components.StateMachine.TurretStates.TURRET_RAISING;
import static org.firstinspires.ftc.teamcode.Components.StateMachine.TurretStates.TURRET_ROTATED;
import static org.firstinspires.ftc.teamcode.Components.StateMachine.TurretStates.TURRET_STRAIGHT;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class StateMachine {
    boolean teleOp = false;
    Logger logger;

    public enum RobotStates {
        IN_WAREHOUSE(false, "IN_WAREHOUSE"),
        SEQUENCING(false, "SEQUENCING"),
        CAROUSEL_SPINNING_CLOCKWISE(false, "CAROUSEL_SPINNING_CLOCKWISE"),
        CAROUSEL_SPINNING_COUNTERCLOCKWISE(false, "CAROUSEL_SPINNING_COUNTERCLOCKWISE"),
        CAROUSEL_STILL(true, "CAROUSEL_STILL"),
        TSE_ARM_DOWN(false, "TSE_ARM_DOWN"),
        TSE_ARM_UP(true, "TSE_ARM_DOWN"),
        TSE_ARM_FIRST_CAP(false, "TSE_ARM_DOWN"),
        TSE_ARM_SECOND_CAP(false, "TSE_ARM_DOWN");

        boolean status;
        String name;

        RobotStates(boolean value, String name) {
            this.status = value;
        }

        public void setStatus(boolean status) {
            this.status = status;
        }
    }

    public enum IntakeStates {
        INTAKE_SWITCHED(false, "INTAKE_SWITCHED"),
        INTAKE_DOWN(false, "INTAKE_DOWN"),
        INTAKING(false, "INTAKING"),
        INTAKE_REVERSING(false, "INTAKE_REVERSING"),
        INTAKE_FLIPPING_UP(false, "INTAKE_FLIPPING_UP"),
        INTAKE_UP(true, "INTAKE_UP"),
        INTAKE_TRANSFERRING(false, "INTAKE_TRANSFERRING"),
        INTAKE_TRANSFERRED(false, "TRANSFERRED"),
        INTAKE_FLIPPING_DOWN(false, "INTAKE_FLIPPING_DOWN");


        boolean status;
        String name;

        IntakeStates(boolean value, String name) {
            this.status = value;
        }

        public void setStatus(boolean status) {
            this.status = status;
        }
    }

    public enum TurretStates {
        SLIDES_EXTENDED(false, "SLIDES_EXTENDED"),
        SLIDES_EXTENDING(false, "SLIDES_EXTENDING"),
        SLIDES_RETRACTED(true, "SLIDES_RETRACTED"),
        SLIDES_RETRACTING(false, "SLIDES_RETRACTING"),
        TURRET_RAISED(false, "TURRET_RAISED"),
        TURRET_RAISING(false, "TURRET_RAISING"),
        TURRET_LOWERING(false, "TURRET_LOWERING"),
        TURRET_FLAT(true, "TURRET_FLAT"),
        TURRET_STRAIGHT(true, "TURRET_STRAIGHT"),
        TURRET_ROTATING_CLOCKWISE(false, "TURRET_ROTATING_CLOCKWISE"),
        TURRET_ROTATING_COUNTER_CLOCKWISE(false, "TURRET_ROTATING_CLOCKWISE"),
        TURRET_ROTATED(false, "TURRET_ROTATED");
        boolean status;
        String name;

        TurretStates(boolean value, String name) {
            this.status = value;
        }

        public void setStatus(boolean status) {
            this.status = status;
        }
    }

    public enum BasketBasketArmStates {
        BASKET_TRANSFER(true, "BASKET_TRANSFER"),
        BASKET_CEILING(false, "BASKET_CEILING"),
        BASKET_DROP(false, "BASKET_DROP"),
        BASKET_ARM_REST(true, "BASKET_ARM_REST"),
        BASKET_ARM_ALLIANCE(true, "BASKET_ARM_ALLIANCE"),
        BASKET_ARM_SHARED(true, "BASKET_ARM_REST"),
        DROPPED(false, "DROPPED");
        boolean status;
        String name;

        BasketBasketArmStates(boolean value, String name) {
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
            INTAKE_DOWN.setStatus(true);
        }
        logger.createFile("SequencingStates", "Runtime,State,Value");
    }

    public void setState(RobotStates state, boolean value) {
        if(state.status!=value) {
            state.setStatus(value);

            if (value == true) {
                logger.log("SequencingStates", "STATE:" + state.values()[1] + ",true");
            } else {
                logger.log("SequencingStates", "STATE:" + state.values()[1] + ",false");
            }
        }
    }

    public void setState(IntakeStates state, boolean value) {
        if(state.status!=value) {
            state.setStatus(value);

            if (value == true) {
                logger.log("SequencingStates", "STATE:" + state.values()[1] + ",true");
            } else {
                logger.log("SequencingStates", "STATE:" + state.values()[1] + ",false");
            }

        }
    }
    public void setState(TurretStates state, boolean value) {
        if(state.status!=value) {
            state.setStatus(value);
            if (value == true) {
                logger.log("SequencingStates", "STATE:" + state.values()[1] + ",true");
            } else {
                logger.log("SequencingStates", "STATE:" + state.values()[1] + ",false");
            }
        }
    }
    public void setState(BasketBasketArmStates state, boolean value) {
        if(state.status!=value) {
            state.setStatus(value);
            if (value == true) {
                logger.log("SequencingStates", "STATE:" + state.values()[1] + ",true");
            } else {
                logger.log("SequencingStates", "STATE:" + state.values()[1] + ",false");
            }
        }
    }

    public boolean getState(RobotStates state) {
        return state.status;
    }
    public boolean getState(IntakeStates state) {
        return state.status;
    }
    public boolean getState(TurretStates state) {
        return state.status;
    }
    public boolean getState(BasketBasketArmStates state) {
        return state.status;
    }

    public boolean checkIf(RobotStates state) {
        return true;
    }

    public boolean checkIf(IntakeStates state) {
        if (state == INTAKING) {
            return (teleOp || IN_WAREHOUSE.status) && !INTAKING.status;
        }
        else if (state == INTAKE_REVERSING) {
            return (!INTAKE_FLIPPING_UP.status && !INTAKE_FLIPPING_DOWN.status && !INTAKE_REVERSING.status);
        }
        else if (state == INTAKE_FLIPPING_UP) {
            return TURRET_STRAIGHT.status && !SLIDES_EXTENDED.status && !TURRET_RAISED.status && BASKET_TRANSFER.status && !INTAKE_FLIPPING_UP.status;
        }
        else if (state == INTAKE_FLIPPING_DOWN) {
            return !INTAKE_UP.status;
        }
        else if (state == INTAKE_TRANSFERRING) {
            return !INTAKE_TRANSFERRING.status && !INTAKE_FLIPPING_UP.status && !INTAKE_FLIPPING_DOWN.status && TURRET_STRAIGHT.status && !SLIDES_EXTENDED.status && !TURRET_RAISED.status && BASKET_TRANSFER.status && !StateMachine.IntakeStates.INTAKE_DOWN.status && SEQUENCING.status;
        }
        else if (state == INTAKE_TRANSFERRED) {
            return INTAKE_TRANSFERRING.status;
        }
        else if (state == INTAKE_DOWN) {
            return !INTAKE_SWITCHED.status;
        }
        else if (state == INTAKE_UP) {
            return !INTAKE_DOWN.status;
        }
        else if (state == INTAKE_SWITCHED) {
            return true;
        }
        else {
            return true;
        }
    }

    public boolean checkIf(TurretStates state) {
        if (state == SLIDES_EXTENDED) {
            return TURRET_ROTATED.status || TURRET_RAISED.status;
        }
        else if (state == SLIDES_EXTENDING) {
            return INTAKE_TRANSFERRED.status && !SLIDES_EXTENDING.status;
        }
        else if (state == SLIDES_RETRACTING) {
            return SLIDES_EXTENDED.status && BASKET_CEILING.status && !SLIDES_RETRACTING.status;
        }
        else if (state == SLIDES_RETRACTED) {
            return BASKET_TRANSFER.status; //TURRET_STRAIGHT.status || TURRET_FLAT.status;
        }
        else if (state == TURRET_RAISING) {
            return INTAKE_TRANSFERRED.status && !TURRET_RAISING.status;
        }
        else if (state == TURRET_RAISED) {
            return BASKET_DROP.status;
        }

        else if (state == TURRET_LOWERING) {
            return BASKET_TRANSFER.status && TURRET_RAISED.status && !TURRET_LOWERING.status;
        }

        else if (state == TURRET_FLAT) {
            return BASKET_TRANSFER.status;
        }

        else if (state == TURRET_STRAIGHT) {
            return BASKET_TRANSFER.status;
        }


        else {
            return true;
        }
    }

    public boolean checkIf(BasketBasketArmStates state) {
        if (state == BASKET_CEILING) {
            return (BASKET_DROP.status && DROPPED.status) || (BASKET_TRANSFER.status && INTAKE_TRANSFERRED.status);
        }
        else if (state == BASKET_TRANSFER) {
            return BASKET_ARM_REST.status && SLIDES_RETRACTED.status && TURRET_STRAIGHT.status;
        }
        else if (state == BASKET_DROP) {
            return (BASKET_ARM_ALLIANCE.status || BASKET_ARM_SHARED.status) && SLIDES_EXTENDED.status;
        }
        else if (state == BASKET_ARM_REST) {
            return BASKET_CEILING.status && SLIDES_RETRACTED.status && TURRET_STRAIGHT.status;
        }
        else if (state == DROPPED) {
            return SLIDES_EXTENDED.status && TURRET_RAISED.status && TURRET_ROTATED.status && (BASKET_ARM_SHARED.status || BASKET_ARM_ALLIANCE.status) && BASKET_CEILING.status;
        }
        else {
            return true;
        }
    }

//    public boolean checkIf(RobotStates state) {
//        if (state == IN_WAREHOUSE) {
//            return ();
//        }
//    }
}
