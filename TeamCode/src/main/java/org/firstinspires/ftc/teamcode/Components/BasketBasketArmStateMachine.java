//package org.firstinspires.ftc.teamcode.Components;
//
//import static org.firstinspires.ftc.teamcode.Components.BasketBasketArmStateMachine.States.BASKET_ARM_ALLIANCE;
//import static org.firstinspires.ftc.teamcode.Components.BasketBasketArmStateMachine.States.BASKET_ARM_REST;
//import static org.firstinspires.ftc.teamcode.Components.BasketBasketArmStateMachine.States.BASKET_ARM_SHARED;
//import static org.firstinspires.ftc.teamcode.Components.BasketBasketArmStateMachine.States.BASKET_CEILING;
//import static org.firstinspires.ftc.teamcode.Components.BasketBasketArmStateMachine.States.BASKET_DROP;
//import static org.firstinspires.ftc.teamcode.Components.BasketBasketArmStateMachine.States.BASKET_TRANSFER;
//import static org.firstinspires.ftc.teamcode.Components.BasketBasketArmStateMachine.States.DROPPED;
//import static org.firstinspires.ftc.teamcode.Components.BasketBasketArmStateMachine.States.TRANSFERRED;
//import static org.firstinspires.ftc.teamcode.Components.IntakeStateMachine.States.INTAKE_TRANSFERRING;
//import static org.firstinspires.ftc.teamcode.Components.TurretStateMachine.States.SLIDES_EXTENDED;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//public class BasketBasketArmStateMachine {
//    boolean teleOp = false;
//    Logger logger;
//
//    public enum States {
//        BASKET_TRANSFER(true, "BASKET_TRANSFER"),
//        BASKET_CEILING(false, "BASKET_CEILING"),
//        BASKET_DROP(false, "BASKET_DROP"),
//        BASKET_ARM_REST(true, "BASKET_ARM_REST"),
//        BASKET_ARM_ALLIANCE(true, "BASKET_ARM_ALLIANCE"),
//        BASKET_ARM_SHARED(true, "BASKET_ARM_REST"),
//        TRANSFERRED(false, "TRANSFERRED"),
//        DROPPED(false, "DROPPED");
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
//    public BasketBasketArmStateMachine(LinearOpMode op, boolean isTeleOp, Logger log) {
//        logger = log;
//        teleOp = isTeleOp;
////        if (teleOp) {
////            States.INTAKE_DOWN.setStatus(true);
////        }
//    }
//
//    public void setState(States state, boolean value) {
//        if(state.status!=value) {
//            state.setStatus(value);
//            if (value == true) {
//                logger.log("STATE:" + state.values()[1] + "true");
//            } else {
//                logger.log("STATE:" + state.values()[1] + "false");
//            }
//        }
//    }
//
//    public boolean getState(States state) {
//        return state.status;
//    }
//    public boolean checkIf(States state) {
//        if (state == BASKET_CEILING) {
//            return (BASKET_DROP.status && DROPPED.status) || (BASKET_TRANSFER.status && TRANSFERRED.status);
//        }
//        else if (state == BASKET_TRANSFER) {
//            return BASKET_ARM_REST.status && !SLIDES_EXTENDED.status;
//        }
//        else if (state == BASKET_DROP) {
//            return (BASKET_ARM_ALLIANCE.status || BASKET_ARM_SHARED.status) && TRANSFERRED.status;
//        }
//        else if (state == BASKET_ARM_REST) {
//            return DROPPED.status;
//        }
//        else if (state == TRANSFERRED) {
//            return INTAKE_TRANSFERRING.status;
//        }
//        else if (state == DROPPED) {
//
//        }
//        else {
//            return true;
//        }
//    }
//}
