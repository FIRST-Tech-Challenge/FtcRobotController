package org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc;


import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.BasketArmStates.BASKET_ARM_ALLIANCE;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.BasketArmStates.BASKET_ARM_REST;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.BasketArmStates.BASKET_ARM_SHARED;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.BasketStates.BASKET_CEILING;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.BasketStates.BASKET_DROP;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.BasketStates.BASKET_TRANSFER;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.IntakeMotorStates.INTAKE_STILL;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.IntakeServoStates.INTAKE_DOWN;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.IntakeServoStates.INTAKE_FLIPPING_DOWN;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.IntakeServoStates.INTAKE_FLIPPING_UP;

import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.IntakeMotorStates.INTAKE_REVERSING;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.RobotStates.SLIDES_EXTENDED;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.IntakeServoStates.INTAKE_UP;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.IntakeMotorStates.INTAKING;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.RobotStates.TSE_ARM_UP;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.SlidesStates.SLIDES_RETRACTED;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.SlidesStates.SLIDES_RETRACTING;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.TurretAAStates.TURRET_FLAT;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.TurretAAStates.TURRET_RAISED;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.TurretRotationStates.TURRET_ROTATED;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.TurretRotationStates.TURRET_ROTATING_CLOCKWISE;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.TurretRotationStates.TURRET_ROTATING_COUNTER_CLOCKWISE;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.TurretRotationStates.TURRET_STRAIGHT;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;

public class StateMachine {
    boolean teleOp = false;
    IntakeServoStates lastIntakeServoState = INTAKE_DOWN;
    IntakeMotorStates lastIntakeMotorState = INTAKE_STILL;
    TurretRotationStates lastTurretRotationState = TURRET_STRAIGHT;
    SlidesStates lastSlidesState = SLIDES_RETRACTED;
    TurretAAStates lastTurretAAState = TURRET_FLAT;
    BasketStates lastBasketState = BASKET_TRANSFER;
    BasketArmStates lastBasketArmState = BASKET_ARM_REST;

    public enum RobotStates {
        SLIDES_EXTENDED(false, "SLIDES_EXTENDED"),
        INTAKE_SWITCHED(false, "INTAKE_SWITCHED"),
        TRANSFERRING(false, "TRANSFERRING"),
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

    public enum IntakeServoStates {
        INTAKE_DOWN(true, "INTAKE_DOWN"),
        INTAKE_FLIPPING_UP(false, "INTAKE_FLIPPING_UP"),
        INTAKE_FLIPPING_DOWN(false, "INTAKE_FLIPPING_DOWN"),
        INTAKE_UP(false, "INTAKE_UP");

        boolean status;
        String name;

        IntakeServoStates(boolean value, String name) {
            this.status = value;
        }

        public void setStatus(boolean status) {
            this.status = status;
        }
    }


    public enum IntakeMotorStates {

        INTAKE_STILL(true, "INTAKE_STILL"),
        INTAKING(false, "INTAKING"),
        INTAKE_REVERSING(false, "INTAKE_REVERSING");

        boolean status;
        String name;

        IntakeMotorStates(boolean value, String name) {
            this.status = value;
        }

        public void setStatus(boolean status) {
            this.status = status;
        }
    }

    public enum TurretRotationStates {
        TURRET_STRAIGHT(true, "TURRET_STRAIGHT"),
        TURRET_ROTATING_CLOCKWISE(false, "TURRET_ROTATING_CLOCKWISE"),
        TURRET_ROTATING_COUNTER_CLOCKWISE(false, "TURRET_ROTATING_CLOCKWISE"),
        TURRET_ROTATED(false, "TURRET_ROTATED");

        boolean status;
        String name;

        TurretRotationStates(boolean value, String name) {
            this.status = value;
        }

        public void setStatus(boolean status) {
            this.status = status;
        }
    }

    public enum SlidesStates {
//        SLIDES_EXTENDING(false, "SLIDES_EXTENDING"),
        SLIDES_RETRACTED(true, "SLIDES_RETRACTED"),
        SLIDES_RETRACTING(false, "SLIDES_RETRACTING");

        boolean status;
        String name;

        SlidesStates(boolean value, String name) {
            this.status = value;
        }

        public void setStatus(boolean status) {
            this.status = status;
        }
    }

    public enum TurretAAStates {

        TURRET_RAISED(false, "TURRET_RAISED"),
        TURRET_FLAT(true, "TURRET_FLAT");

        boolean status;
        String name;

        TurretAAStates(boolean value, String name) {
            this.status = value;
        }

        public void setStatus(boolean status) {
            this.status = status;
        }
    }

    public enum BasketStates {
        BASKET_TRANSFER(true, "BASKET_TRANSFER"),
        BASKET_CEILING(false, "BASKET_CEILING"),
        BASKET_DROP(false, "BASKET_DROP");

        boolean status;
        String name;

        BasketStates(boolean value, String name) {
            this.status = value;
        }

        public void setStatus(boolean status) {
            this.status = status;
        }
    }

    public enum BasketArmStates {

        BASKET_ARM_REST(true, "BASKET_ARM_REST"),
        BASKET_ARM_ALLIANCE(false, "BASKET_ARM_ALLIANCE"),
        BASKET_ARM_SHARED(false, "BASKET_ARM_REST");

        boolean status;
        String name;

        BasketArmStates(boolean value, String name) {
            this.status = value;
        }

        public void setStatus(boolean status) {
            this.status = status;
        }
    }

    public StateMachine(boolean isTeleOp) {
        teleOp = isTeleOp;
        logger.createFile("SequencingStates", "Runtime,State,Value");
        if (teleOp) {
            setState(INTAKE_DOWN, true);
            setState(TSE_ARM_UP, true);
            setState(INTAKE_STILL, true);
            setState(TURRET_STRAIGHT, true);
            setState(TURRET_FLAT, true);
            setState(SLIDES_RETRACTED, true);
            setState(BASKET_ARM_REST, true);
            setState(BASKET_TRANSFER, true);
        }


    }


    public void setState(RobotStates state, boolean value) {
//        if (checkIf(state)) {
        if (state.status != value) {
            state.setStatus(value);

            if (value) {
                logger.log("SequencingStates", "STATE: " + state + " changed to true");
            } else {
                logger.log("SequencingStates", "STATE: " + state + " changed to false");
            }
        }

//        else if (!checkIf(state) && !state.status) {
//            state.setStatus(false);
//        }
    }


    public void setState(IntakeServoStates state, boolean value) {
        if (checkIf(state)) {
            for (int i = 0; i < IntakeServoStates.values().length; i++) {
                if (IntakeServoStates.values()[i] != state) {
                    IntakeServoStates.values()[i].setStatus(false);
                }
            }
            if (state.status != value) {
                state.setStatus(value);
                lastIntakeServoState = state;

                if (value) {
                    logger.log("SequencingStates", "STATE: " + state + " changed to true");
                } else {
                    logger.log("SequencingStates", "STATE: " + state + " changed to false");
                }
            }
        }
        else if (!checkIf(state) && !state.status) {
            state.setStatus(false);
        }
    }

    public void setState(IntakeMotorStates state, boolean value) {
        if (checkIf(state)) {
            for (int i = 0; i < IntakeMotorStates.values().length; i++) {
                if (IntakeMotorStates.values()[i] != state) {
                    IntakeMotorStates.values()[i].setStatus(false);
                }
            }
            if (state.status != value) {
                state.setStatus(value);
                lastIntakeMotorState = state;

                if (value) {
                    logger.log("SequencingStates", "STATE: " + state + " changed to true");
                } else {
                    logger.log("SequencingStates", "STATE: " + state + " changed to false");
                }
            }
        }
        else if (!checkIf(state) && !state.status) {
            state.setStatus(false);
        }
    }

    public void setState(TurretRotationStates state, boolean value) {
        if (checkIf(state)) {
            for (int i = 0; i < TurretRotationStates.values().length; i++) {
                if (TurretRotationStates.values()[i] != state) {
                    TurretRotationStates.values()[i].setStatus(false);
                }
            }
            if (state.status != value) {
                state.setStatus(value);
                lastTurretRotationState = state;

                if (value) {
                    logger.log("SequencingStates", "STATE: " + state + " changed to true");
                } else {
                    logger.log("SequencingStates", "STATE: " + state + " changed to false");
                }
            }
        }
        else if (!checkIf(state) && !state.status) {
            state.setStatus(false);
        }
    }

    public void setState(SlidesStates state, boolean value) {
        if (checkIf(state)) {
            for (int i = 0; i < SlidesStates.values().length; i++) {
                if (SlidesStates.values()[i] != state) {
                    SlidesStates.values()[i].setStatus(false);
                }
            }
            if (state.status != value) {
                state.setStatus(value);
                lastSlidesState = state;

                if (value) {
                    logger.log("SequencingStates", "STATE: " + state + " changed to true");
                } else {
                    logger.log("SequencingStates", "STATE: " + state + " changed to false");
                }
            }
        }
        else if (!checkIf(state) && !state.status) {
            state.setStatus(false);
        }
    }

    public void setState(TurretAAStates state, boolean value) {
        if (checkIf(state)) {
            for (int i = 0; i < TurretAAStates.values().length; i++) {
                if (TurretAAStates.values()[i] != state) {
                    TurretAAStates.values()[i].setStatus(false);
                }
            }
            if (state.status != value) {
                state.setStatus(value);
                lastTurretAAState = state;

                if (value) {
                    logger.log("SequencingStates", "STATE: " + state + " changed to true");
                } else {
                    logger.log("SequencingStates", "STATE: " + state + " changed to false");
                }
            }
        }
        else if (!checkIf(state) && !state.status) {
            state.setStatus(false);
        }
    }

    public void setState(BasketStates state, boolean value) {
        if (checkIf(state)) {
            for (int i = 0; i < BasketStates.values().length; i++) {
                if (BasketStates.values()[i] != state) {
                    BasketStates.values()[i].setStatus(false);
                }
            }
            if (state.status != value) {
                state.setStatus(value);
                lastBasketState = state;

                if (value) {
                    logger.log("SequencingStates", "STATE: " + state + " changed to true");
                } else {
                    logger.log("SequencingStates", "STATE: " + state + " changed to false");
                }
            }
        }
        else if (!checkIf(state) && !state.status) {
            state.setStatus(false);
        }
    }

    public void setState(BasketArmStates state, boolean value) {
        if (checkIf(state)) {
            for (int i = 0; i < BasketArmStates.values().length; i++) {
                if (BasketArmStates.values()[i] != state) {
                    BasketArmStates.values()[i].setStatus(false);
                }
            }
            if (state.status != value) {
                state.setStatus(value);
                lastBasketArmState = state;

                if (value) {
                    logger.log("SequencingStates", "STATE: " + state + " changed to true");
                } else {
                    logger.log("SequencingStates", "STATE: " + state + " changed to false");
                }
            }
        }
        else if (!checkIf(state) && !state.status) {
            state.setStatus(false);
        }
    }

    public void logCurrentStates() {
        logger.log("SequencingStates", "Last Intake Servo State: " + lastIntakeServoState, true, true);
        logger.log("SequencingStates", "Last Intake MotorState: " + lastIntakeMotorState);
        logger.log("SequencingStates", "Last Turret RotationState: " + lastTurretRotationState);
        logger.log("SequencingStates", "Last Slides State: " + lastSlidesState);
        logger.log("SequencingStates", "Last TurretAA State: " + lastTurretAAState);
        logger.log("SequencingStates", "Last Basket State: " + lastBasketState);
        logger.log("SequencingStates", "Last Basket ArmState: " + lastBasketArmState);
    }

    public boolean getState(RobotStates state) {
        return state.status;
    }
    public boolean getState(IntakeServoStates state) {
        return state.status;
    }
    public boolean getState(IntakeMotorStates state) {
        return state.status;
    }
    public boolean getState(TurretRotationStates state) {
        return state.status;
    }
    public boolean getState(SlidesStates state) {
        return state.status;
    }
    public boolean getState(TurretAAStates state) {
        return state.status;
    }
    public boolean getState(BasketStates state) {
        return state.status;
    }
    public boolean getState(BasketArmStates state) {
        return state.status;
    }

//INTAKE_SWITCHED(false, "INTAKE_SWITCHED"),
//IN_WAREHOUSE(false, "IN_WAREHOUSE"),
//SEQUENCING(false, "SEQUENCING"),
//CAROUSEL_SPINNING_CLOCKWISE(false, "CAROUSEL_SPINNING_CLOCKWISE"),
//CAROUSEL_SPINNING_COUNTERCLOCKWISE(false, "CAROUSEL_SPINNING_COUNTERCLOCKWISE"),
//CAROUSEL_STILL(true, "CAROUSEL_STILL"),
//TSE_ARM_DOWN(false, "TSE_ARM_DOWN"),
//TSE_ARM_UP(true, "TSE_ARM_DOWN"),
//TSE_ARM_FIRST_CAP(false, "TSE_ARM_DOWN"),
//TSE_ARM_SECOND_CAP(false, "TSE_ARM_DOWN");

//    public boolean checkIf(RobotStates state) {
////        if (state == INTAKE_SWITCHED) {
////            return (true);
////        }
////        if (state == SEQUENCING) {
////            return (INTAKE_FLIPPING_UP.status || INTAKE_FLIPPING_DOWN.status || INTAKE_TRANSFERRING.status || SLIDES_EXTENDING.status || BASKET_ARM_ALLIANCE.status || BASKET_ARM_SHARED.status || BASKET_DROP.status);
////        }
//        if (state == CAROUSEL_SPINNING_CLOCKWISE) {
//            return (!CAROUSEL_SPINNING_CLOCKWISE.status);
//        }
//        else if (state == TRANSFERRING) {
//            return (INTAKE_UP.status && !TRANSFERRING.status);
//        }
//        else if (state == CAROUSEL_SPINNING_COUNTERCLOCKWISE) {
//            return (!CAROUSEL_SPINNING_COUNTERCLOCKWISE.status);
//        }
//        else if (state == TSE_ARM_DOWN) {
//            return (TSE_ARM_SECOND_CAP.status);
//        }
//        else if (state == TSE_ARM_UP) {
//            return (TSE_ARM_DOWN.status);
//        }
//        else if (state == TSE_ARM_FIRST_CAP) {
//            return (TSE_ARM_UP.status);
//        }
//        else if (state == TSE_ARM_SECOND_CAP) {
//            return (TSE_ARM_FIRST_CAP.status);
//        }
//        else {
//            return true;
//        }
//    }

    public boolean checkIf(IntakeServoStates state) {
        if (state == INTAKE_DOWN) {
            return (INTAKE_FLIPPING_DOWN.status);
        }
        else if (state == INTAKE_FLIPPING_DOWN) {
            return (INTAKE_FLIPPING_UP.status || INTAKE_UP.status);
        }
        else if (state == INTAKE_FLIPPING_UP) {
            return (INTAKE_FLIPPING_DOWN.status || INTAKE_DOWN.status);
        }
        else if (state == INTAKE_UP) {
            return (INTAKE_FLIPPING_UP.status);
        }
        else {
            return true;
        }
    }

    public boolean checkIf(IntakeMotorStates state) {
        if (state == INTAKE_STILL) {
            return (!INTAKE_STILL.status);
        }
        else if (state == INTAKE_REVERSING) {
            return (INTAKE_DOWN.status && !INTAKE_REVERSING.status);
        }

        else if (state == INTAKING) {
            return (INTAKE_DOWN.status || INTAKE_FLIPPING_UP.status);
        }
        else {
            return true;
        }
    }

    public boolean checkIf(TurretRotationStates state) {
        if (state == TURRET_STRAIGHT) {
            return (TURRET_ROTATING_COUNTER_CLOCKWISE.status || TURRET_ROTATING_CLOCKWISE.status);
        }
        else if (state == TURRET_ROTATING_CLOCKWISE) {
            return (!TURRET_ROTATING_CLOCKWISE.status);
        }
        else if (state == TURRET_ROTATING_COUNTER_CLOCKWISE) {
            return (!TURRET_ROTATING_COUNTER_CLOCKWISE.status);
        }
        else if (state == TURRET_ROTATED) {
            return (TURRET_ROTATING_COUNTER_CLOCKWISE.status || TURRET_ROTATING_CLOCKWISE.status);
        }
        else {
            return true;
        }
    }

    public boolean checkIf(SlidesStates state) {
//        if (state == SLIDES_EXTENDED) {
//            return (SLIDES_RETRACTING.status || SLIDES_RETRACTED.status);
//        }
        if (state == SLIDES_RETRACTED) {
            return (SLIDES_RETRACTING.status);
        }
        else if (state == SLIDES_RETRACTING) {
            return (SLIDES_EXTENDED.status);
        }
        else {
            return true;
        }
    }

    public boolean checkIf(TurretAAStates state) {
        if (state == TURRET_FLAT) {
            return (!TURRET_FLAT.status);
        }
        else if (state == TURRET_RAISED) {
            return (!TURRET_RAISED.status);
        }
        else {
            return true;
        }
    }

    public boolean checkIf(BasketStates state) {
        if (state == BASKET_CEILING) {
            return (!BASKET_CEILING.status);
        }
        else if (state == BASKET_TRANSFER) {
            return (BASKET_CEILING.status);
        }
        else if (state == BASKET_DROP) {
            return (BASKET_CEILING.status);
        }
        else {
            return true;
        }
    }

    public boolean checkIf(BasketArmStates state) {
        if (state == BASKET_ARM_REST) {
            return (!BASKET_ARM_REST.status);
        }
        else if (state == BASKET_ARM_ALLIANCE) {
            return (BASKET_ARM_REST.status);
        }
        else if (state == BASKET_ARM_SHARED) {
            return (BASKET_ARM_REST.status);
        }
        else {
            return true;
        }
    }
}
