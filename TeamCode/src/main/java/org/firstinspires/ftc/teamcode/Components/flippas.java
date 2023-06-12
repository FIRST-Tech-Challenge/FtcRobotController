package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Components.Claw.ClawStates.CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.Components.Claw.ClawStates.CLAW_WIDE;
import static org.firstinspires.ftc.teamcode.Components.LiftArm.liftArmStates.ARM_INTAKE;
import static org.firstinspires.ftc.teamcode.Components.LiftArm.liftArmStates.ARM_LOWERING;
import static org.firstinspires.ftc.teamcode.Components.LiftArm.liftArmStates.ARM_OUTTAKE;
import static org.firstinspires.ftc.teamcode.Components.LiftArm.liftArmStates.ARM_RAISING;
//import static org.firstinspires.ftc.teamcode.Components.Switch.SwitchStates.PRESSED;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFDualServo;
@Config
public class flippas {

    private RFDualServo flippaServo;

    private final double LIFT_ARM_INTAKE_POS = 0.3;
    private final double LIFT_ARM_CYCLE_POS = 0.955;

    private final double LIFT_ARM_OUTTAKE_POS = 0.78;
    public static double LIFT_ARM_FLIP_POS = 0.5;

    public double liftArmServoLastSwitchTime = 0;
    //temporary
    public final double LIFT_ARM_SERVO_SWITCH_TIME = 0.5;
    double liftPos =0.0;

    //States:
    //ARM_INTAKE
    //ARM_RAISING
    //ARM_LOWERING
    //ARM_OUTTAKE

    public enum flippaStates {

        FLIP_INTAKE(true, "FLIP_INTAKE"),
        FLIP_RAISING(false, "FLIP_RAISING"),
        FLIP_LOWERING(false, "FLIP_LOWERING"),
        FLIP_OUTTAKE(false, "FLIP_OUTTAKE");

        boolean status;
        String name;

        flippaStates(boolean p_status, String p_name) {
            this.status = p_status;
            this.name = p_name;
        }

        public void setStatus(boolean p_status) {
            this.status = p_status;
            if(p_status) {
                for (int i = 0; i < flippaStates.values().length; i++) {
                    if (flippaStates.values()[i] != this) {
                        flippaStates.values()[i].status = false;
                    }
                }
            }
        }

        public boolean getStatus() {
            return this.status;
        }
    }

    //constructor
    public flippas() {
        //init RFServo
        flippaServo = new RFDualServo("rightFlipper", "leftFlipper", 1);
        //set servo position
        flippaServo.setPositions(LIFT_ARM_INTAKE_POS);

        //set state of claw open to true
        flippaStates.FLIP_OUTTAKE.setStatus(true);

        flippaServo.setLasttime(-0.5);

    }

    public void update() {
        if (flippaStates.FLIP_LOWERING.status && time - flippaServo.getLastTime() > LIFT_ARM_SERVO_SWITCH_TIME) {
            flippaStates.FLIP_INTAKE.setStatus(true);

        }
        if (flippaStates.FLIP_RAISING.status && time- flippaServo.getLastTime() > LIFT_ARM_SERVO_SWITCH_TIME) {
            flippaStates.FLIP_OUTTAKE.setStatus(true);
//            flippaServo.disableServos();
//            flippaServo.disableServos();
        }

    }

    public void toggleArmPosition() {
        flippaServo.flipServosInterval(LIFT_ARM_INTAKE_POS, LIFT_ARM_OUTTAKE_POS);
        logger.log("/RobotLogs/GeneralRobot", "liftArmDualServo,toggleArmPosition()"
                + ",Lift Arm Toggled", true);
    }


    //lower arm to intake position
    public void lowerFlippas() {
        //no input

        //the state of claw open has to be true (cone has already been dropped)
        //&& LIFT_GROUND.status
        if (/*CLAW_OPEN.status && */flippaStates.FLIP_OUTTAKE.status) {

            //set servo position
            flippaServo.setPositions(LIFT_ARM_INTAKE_POS);

            //set state of claw open to true
            flippaStates.FLIP_LOWERING.setStatus(true);
            liftPos = LIFT_ARM_INTAKE_POS;
            liftArmServoLastSwitchTime = time;

            //log to general robot log that the claw has been opened through function openClaw()
            logger.log("/RobotLogs/GeneralRobot", flippaServo.getDeviceName() + ",lowerFlippas()"
                    + ",Flippa Lowered to Intake Position", true);
        }
    }
    public boolean isCylce(){
        if(liftPos == LIFT_ARM_CYCLE_POS){
            return true;
        }else{
            return false;
        }
    }
    public void flipCone(){
        if(time>liftArmServoLastSwitchTime+LIFT_ARM_SERVO_SWITCH_TIME) {
            flippaServo.setPositions(LIFT_ARM_FLIP_POS);
        }
    }
    public void cycleLiftArmToCylce(){
        if (liftPos != LIFT_ARM_CYCLE_POS) {

            //set servo position
            flippaServo.setPositions(LIFT_ARM_CYCLE_POS);

            ARM_INTAKE.setStatus(true);
            liftPos = LIFT_ARM_CYCLE_POS;

            //log to general robot log that the claw has been opened through function openClaw()
            logger.log("/RobotLogs/GeneralRobot", flippaServo.getDeviceName() + ",lowerLiftArmToIntake()"
                    + ",Lift Arm Lowered to Intake Position", true);
        }
    }

    //lower arm to intake position
    public void raiseLiftArmToOuttake() {
        //no input

        //the state of claw closed has to be true (cone has already been grabbed)
        // && (LIFT_GROUND_JUNCTION.status || LIFT_LOW.status || LIFT_MID.status || LIFT_HIGH.status)
        if ( /*&&PRESSED.getStatus()*/  flippaStates.FLIP_INTAKE.status && time-flippaServo.getLastTime()>0.2) {

            //set servo position
            flippaServo.setPositions(LIFT_ARM_OUTTAKE_POS);

            //set state of claw open to true
            flippaStates.FLIP_RAISING.setStatus(true);

            //log to general robot log that the claw has been opened through function openClaw()
            logger.log("/RobotLogs/GeneralRobot", flippaServo.getDeviceName() + ",raiseLiftArmToOuttake()"
                    + ",Lift Arm Raised to Outtake Position", true);
            liftPos = LIFT_ARM_OUTTAKE_POS;
        }
        else {
            logger.log("/RobotLogs/GeneralRobot", flippaServo.getDeviceName() + ",raiseLiftArmToOuttake()"
                    + "ARM_INTAKE = " + ARM_INTAKE.status + "ARM_OUTTAKE =" + ARM_OUTTAKE.status, true);
        }
    }

    public void disableArm() {
        flippaServo.disableServos();
    }
    public void enableArm(){
        flippaServo.enableServos();
    }
    public boolean ableArmed(){
        return flippaServo.abledServos();
    }

}

