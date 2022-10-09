package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Components.Claw.ClawStates.CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.Components.Claw.ClawStates.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.Components.LiftArm.liftArmStates.ARM_INTAKE;
import static org.firstinspires.ftc.teamcode.Components.LiftArm.liftArmStates.ARM_OUTTAKE;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;
import org.firstinspires.ftc.teamcode.Old.Components.Misc.ColorDistanceRevV3;
import org.firstinspires.ftc.teamcode.Old.Components.Misc.StateMachine;

public class LiftArm {

    private RFServo liftArmServo;

    private final double liftArmIntakePos = 0.1;

    private final double liftArmOuttakePos = 0.4;

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

        liftArmStates(boolean value, String name) {
            this.status = value;
            this.name = name;
        }

        public void setStatus(boolean status) {
            this.status = status;
            if(status) {
                for (int i = 0; i < liftArmStates.values().length; i++) {
                    if (liftArmStates.values()[i].status != status) {
                        liftArmStates.values()[i].setStatus(false);
                    }
                }
            }
        }
    }

    //constructor
    public LiftArm() {
        //init RFServo
        liftArmServo = new RFServo("liftArmServo", 0.5);

    }

    public void toggleArmPosition() {
        liftArmServo.flipServoInterval(liftArmIntakePos, liftArmOuttakePos);
    }


    //lower arm to intake position
    public void lowerLiftArmToIntake() {
        //no input

        //the state of claw open has to be true (cone has already been dropped)
//        if (CLAW_OPEN.status) {

        //set servo position
        liftArmServo.setPosition(liftArmIntakePos);

        //set state of claw open to true
        ARM_INTAKE.setStatus(true);

        //log to general robot log that the claw has been opened through function openClaw()
        logger.log("/RobotLogs/GeneralRobot", liftArmServo.getDeviceName() + ",lowerLiftArmToIntake()"
                + ",Lift Arm Lowered to Intake Position", true);
//        }
    }

    //lower arm to intake position
    public void raiseLiftArmToOuttake() {
        //no input

        //the state of claw closed has to be true (cone has already been grabbed)
//        if (CLAW_CLOSED.status) {

        //set servo position
        liftArmServo.setPosition(liftArmOuttakePos);

        //set state of claw open to true
        ARM_OUTTAKE.setStatus(true);

        //log to general robot log that the claw has been opened through function openClaw()
        logger.log("/RobotLogs/GeneralRobot", liftArmServo.getDeviceName() + ",raiseLiftArmToOuttake()"
                + ",Lift Arm Raised to Outtake Position", true);
//        }
    }

}

