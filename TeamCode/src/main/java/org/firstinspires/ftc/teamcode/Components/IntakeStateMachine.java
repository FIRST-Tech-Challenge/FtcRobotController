//package org.firstinspires.ftc.teamcode.Components;
//
//import static org.firstinspires.ftc.teamcode.Components.BasketBasketArmStateMachine.States.BASKET_TRANSFER;
//import static org.firstinspires.ftc.teamcode.Components.IntakeStateMachine.States.INTAKE_DOWN;
//import static org.firstinspires.ftc.teamcode.Components.IntakeStateMachine.States.INTAKE_FLIPPING_DOWN;
//import static org.firstinspires.ftc.teamcode.Components.IntakeStateMachine.States.INTAKE_FLIPPING_UP;
//import static org.firstinspires.ftc.teamcode.Components.IntakeStateMachine.States.INTAKE_UP;
//import static org.firstinspires.ftc.teamcode.Components.IntakeStateMachine.States.INTAKING;
//import static org.firstinspires.ftc.teamcode.Components.IntakeStateMachine.States.INTAKE_REVERSING;
//import static org.firstinspires.ftc.teamcode.Components.IntakeStateMachine.States.INTAKE_TRANSFERRING;
//
//import static org.firstinspires.ftc.teamcode.Components.StateMachine.States.SEQUENCING;
//import static org.firstinspires.ftc.teamcode.Components.TurretStateMachine.States.SLIDES_EXTENDED;
//import static org.firstinspires.ftc.teamcode.Components.StateMachine.States.IN_WAREHOUSE;
//import static org.firstinspires.ftc.teamcode.Components.IntakeStateMachine.States.INTAKE_SWITCHED;
//import static org.firstinspires.ftc.teamcode.Components.TurretStateMachine.States.TURRET_RAISED;
//import static org.firstinspires.ftc.teamcode.Components.TurretStateMachine.States.TURRET_STRAIGHT;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//public class IntakeStateMachine {
//    boolean teleOp = false;
//    Logger logger;
//
//    public enum States {
//        INTAKE_SWITCHED(false, "INTAKE_SWITCHED"),
//        INTAKE_DOWN(false, "INTAKE_DOWN"),
//        INTAKING(false, "INTAKING"),
//        INTAKE_REVERSING(false, "INTAKE_REVERSING"),
//        INTAKE_FLIPPING_UP(false, "INTAKE_FLIPPING_UP"),
//        INTAKE_UP(false, "INTAKE_UP"),
//        INTAKE_TRANSFERRING(false, "INTAKE_TRANSFERRING"),
//        INTAKE_TRANSFERRED(false, "TRANSFERRED"),
//        INTAKE_FLIPPING_DOWN(false, "INTAKE_FLIPPING_DOWN");
//
//        boolean status;
//        String name;
//
//        States(boolean value, String name) {
//            this.status = value;
//        }
//
//        public void setStatus(boolean status) {
//            this.status = status;
//        }
//    }
//
//    public IntakeStateMachine(LinearOpMode op, boolean isTeleOp, Logger log) {
//        logger = log;
//        teleOp = isTeleOp;
//        if (teleOp) {
//            StateMachine.States.INTAKE_DOWN.setStatus(true);
//        }
//    }
//
//    public void setState(IntakeStateMachine.States state, boolean value) {
//        if(state.status!=value) {
//            state.setStatus(value);
//            if (value) {
//                logger.log("STATE:" + state.values()[1] + "true");
//            } else {
//                logger.log("STATE:" + state.values()[1] + "false");
//            }
//        }
//    }
//
//    public boolean getState(IntakeStateMachine.States state) {
//        return state.status;
//    }
//
//    public boolean checkIf(IntakeStateMachine.States state) {
//        if (state == INTAKING) {
//            return (teleOp || IN_WAREHOUSE.status) && !INTAKING.status;
//        }
//        else if (state == INTAKE_REVERSING) {
//            return (!INTAKE_FLIPPING_UP.status && !INTAKE_FLIPPING_DOWN.status && !INTAKE_REVERSING.status);
//        }
//        else if (state == INTAKE_FLIPPING_UP) {
//            return TURRET_STRAIGHT.status && !SLIDES_EXTENDED.status && !TURRET_RAISED.status && BASKET_TRANSFER.status && !INTAKE_FLIPPING_UP.status;
//        }
//        else if (state == INTAKE_FLIPPING_DOWN) {
//            return !INTAKE_UP.status;
//        }
//        else if (state == INTAKE_TRANSFERRING) {
//            return !INTAKE_TRANSFERRING.status && !INTAKE_FLIPPING_UP.status && !INTAKE_FLIPPING_DOWN.status && TURRET_STRAIGHT.status && !SLIDES_EXTENDED.status && !TURRET_RAISED.status && BASKET_TRANSFER.status && !StateMachine.States.INTAKE_DOWN.status && SEQUENCING.status;
//        }
//        else if (state == INTAKE_DOWN) {
//            return !INTAKE_SWITCHED.status;
//        }
//        else if (state == INTAKE_UP) {
//            return !INTAKE_DOWN.status;
//        }
//        else if (state == INTAKE_SWITCHED) {
//            return true;
//        }
//        else {
//            return true;
//        }
//    }
//}
