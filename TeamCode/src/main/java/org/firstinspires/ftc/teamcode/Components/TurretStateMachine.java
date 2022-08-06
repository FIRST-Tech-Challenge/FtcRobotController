//package org.firstinspires.ftc.teamcode.Components;
//
//import static org.firstinspires.ftc.teamcode.Components.IntakeStateMachine.States.INTAKE_TRANSFERRED;
//import static org.firstinspires.ftc.teamcode.Components.StateMachine.States.BASKET_ARM_REST;
//import static org.firstinspires.ftc.teamcode.Components.StateMachine.States.BASKET_DROP;
//import static org.firstinspires.ftc.teamcode.Components.StateMachine.States.BASKET_TRANSFER;
//import static org.firstinspires.ftc.teamcode.Components.StateMachine.States.DROPPED;
//import static org.firstinspires.ftc.teamcode.Components.StateMachine.States.FLIPPING;
//import static org.firstinspires.ftc.teamcode.Components.StateMachine.States.INTAKING;
//import static org.firstinspires.ftc.teamcode.Components.StateMachine.States.IN_WAREHOUSE;
//import static org.firstinspires.ftc.teamcode.Components.StateMachine.States.SEQUENCING;
//import static org.firstinspires.ftc.teamcode.Components.StateMachine.States.TRANSFERRED;
//import static org.firstinspires.ftc.teamcode.Components.StateMachine.States.TURRET_STRAIGHT;
//import static org.firstinspires.ftc.teamcode.Components.TurretStateMachine.States.SLIDES_EXTENDED;
//import static org.firstinspires.ftc.teamcode.Components.TurretStateMachine.States.SLIDES_EXTENDING;
//import static org.firstinspires.ftc.teamcode.Components.TurretStateMachine.States.SLIDES_RETRACTED;
//import static org.firstinspires.ftc.teamcode.Components.TurretStateMachine.States.SLIDES_RETRACTING;
//import static org.firstinspires.ftc.teamcode.Components.TurretStateMachine.States.TURRET_FLAT;
//import static org.firstinspires.ftc.teamcode.Components.TurretStateMachine.States.TURRET_LOWERING;
//import static org.firstinspires.ftc.teamcode.Components.TurretStateMachine.States.TURRET_RAISED;
//import static org.firstinspires.ftc.teamcode.Components.TurretStateMachine.States.TURRET_RAISING;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//public class TurretStateMachine {
//    boolean teleOp = false;
//    Logger logger;
//
//    public enum States {
//        SLIDES_EXTENDED(false, "SLIDES_EXTENDED"),
//        SLIDES_EXTENDING(false, "SLIDES_EXTENDING"),
//        SLIDES_RETRACTED(true, "SLIDES_RETRACTED"),
//        SLIDES_RETRACTING(false, "SLIDES_RETRACTING"),
//        TURRET_RAISED(false, "TURRET_RAISED"),
//        TURRET_RAISING(false, "TURRET_RAISING"),
//        TURRET_LOWERING(false, "TURRET_LOWERING"),
//        TURRET_FLAT(true, "TURRET_FLAT"),
//        TURRET_STRAIGHT(true, "TURRET_STRAIGHT"),
//        TURRET_ROTATING_CLOCKWISE(false, "TURRET_ROTATING_CLOCKWISE"),
//        TURRET_ROTATING_COUNTER_CLOCKWISE(false, "TURRET_ROTATING_CLOCKWISE"),
//        TURRET_ROTATED(false, "TURRET_ROTATED");
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
//    public TurretStateMachine(LinearOpMode op, boolean isTeleOp, Logger log) {
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
//
//    public boolean checkIf(States state) {
//        if (state == SLIDES_EXTENDED) {
//            return !BASKET_TRANSFER.status || !BASKET_ARM_REST.status;
//        }
//        else if (state == SLIDES_EXTENDING) {
//            return INTAKE_TRANSFERRED.status;
//        }
//        else if (state == SLIDES_RETRACTING) {
//            return SLIDES_EXTENDED.status && BASKET_DROP.status;
//        }
////        else if (state == SLIDES_RETRACTED) {
////            return
////        }
//        else if (state == TURRET_RAISING) {
//            return INTAKE_TRANSFERRED.status;
//        }
////        else if (state == TURRET_RAISED) {
////            return
////        }
//
//        else if (state == TURRET_LOWERING) {
//            return BASKET_TRANSFER.status && TURRET_RAISED.status;
//        }
//
////        else if (state == TURRET_FLAT) {
////            return
////        }
//
////        else if (state == TURRET_STRAIGHT) {
////            return
////        }
//
//
//        else {
//            return true;
//        }
//    }
//}
