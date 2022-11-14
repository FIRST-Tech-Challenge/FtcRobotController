package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Components.LiftArm.liftArmStates.ARM_INTAKE;
import static org.firstinspires.ftc.teamcode.Components.LiftArm.liftArmStates.ARM_LOWERING;
import static org.firstinspires.ftc.teamcode.Components.LiftArm.liftArmStates.ARM_OUTTAKE;
import static org.firstinspires.ftc.teamcode.Components.LiftArm.liftArmStates.ARM_RAISING;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFDualServo;

public class LiftArm {

    private RFDualServo liftArmServo;

    private final double LIFT_ARM_INTAKE_POS = 0.13;

    private final double LIFT_ARM_OUTTAKE_POS = 0.57;

    public double liftArmServoLastSwitchTime = 0;
    //temporary
    public final double LIFT_ARM_SERVO_SWITCH_TIME = 0.2;

    //States:
    //ARM_INTAKE
    //ARM_RAISING
    //ARM_LOWERING
    //ARM_OUTTAKE

    public enum liftArmStates {

        ARM_INTAKE(true, "ARM_INTAKE"),
        ARM_RAISING(false, "ARM_RAISING"),
        ARM_LOWERING(false, "ARM_LOWERING"),
        ARM_OUTTAKE(false, "ARM_OUTTAKE");

        boolean status;
        String name;

        liftArmStates(boolean p_status, String p_name) {
            this.status = p_status;
            this.name = p_name;
        }

        public void setStatus(boolean p_status) {
            this.status = p_status;
            if(p_status) {
                for (int i = 0; i < liftArmStates.values().length; i++) {
                    if (liftArmStates.values()[i] != this) {
                        liftArmStates.values()[i].status = false;
                    }
                }
            }
        }

        public boolean getStatus() {
            return this.status;
        }
    }

    //constructor
    public LiftArm() {
        //init RFServo
        liftArmServo = new RFDualServo("liftArmServo", "liftArmServo2", 0.7);
        liftArmServo.setPositions(LIFT_ARM_INTAKE_POS);
        ARM_INTAKE.setStatus(true);

    }

    public void updateLiftArmStates() {
        if (ARM_LOWERING.status && op.getRuntime() - liftArmServo.getLastTime() > LIFT_ARM_SERVO_SWITCH_TIME) {
            ARM_INTAKE.setStatus(true);
        }
        if (ARM_RAISING.status && op.getRuntime() - liftArmServo.getLastTime() > LIFT_ARM_SERVO_SWITCH_TIME) {
            ARM_OUTTAKE.setStatus(true);
        }
    }

    public void toggleArmPosition() {
        liftArmServo.flipServosInterval(LIFT_ARM_INTAKE_POS, LIFT_ARM_OUTTAKE_POS);
        logger.log("/RobotLogs/GeneralRobot", "liftArmDualServo,toggleArmPosition()"
                + ",Lift Arm Toggled", true);
    }


    //lower arm to intake position
    public void lowerLiftArmToIntake() {
        //no input

        //the state of claw open has to be true (cone has already been dropped)
        //&& LIFT_GROUND.status
        if (/*CLAW_OPEN.status && */ARM_OUTTAKE.status) {

            //set servo position
            liftArmServo.setPositions(LIFT_ARM_INTAKE_POS);

            //set state of claw open to true
            ARM_LOWERING.setStatus(true);

            //log to general robot log that the claw has been opened through function openClaw()
            logger.log("/RobotLogs/GeneralRobot", liftArmServo.getDeviceName() + ",lowerLiftArmToIntake()"
                    + ",Lift Arm Lowered to Intake Position", true);
        }
    }

    //lower arm to intake position
    public void raiseLiftArmToOuttake() {
        //no input

        //the state of claw closed has to be true (cone has already been grabbed)
        // && (LIFT_GROUND_JUNCTION.status || LIFT_LOW.status || LIFT_MID.status || LIFT_HIGH.status)
        if ( ARM_INTAKE.status) {

            //set servo position
            liftArmServo.setPositions(LIFT_ARM_OUTTAKE_POS);

            //set state of claw open to true
            ARM_RAISING.setStatus(true);

            //log to general robot log that the claw has been opened through function openClaw()
            logger.log("/RobotLogs/GeneralRobot", liftArmServo.getDeviceName() + ",raiseLiftArmToOuttake()"
                    + ",Lift Arm Raised to Outtake Position", true);
        }
        else {
            logger.log("/RobotLogs/GeneralRobot", liftArmServo.getDeviceName() + ",raiseLiftArmToOuttake()"
                    + "ARM_INTAKE = " + ARM_INTAKE.status + "ARM_OUTTAKE =" + ARM_OUTTAKE.status, true);
        }
    }

}

