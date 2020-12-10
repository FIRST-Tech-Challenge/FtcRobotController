package org.firstinspires.ftc.teamcode.SubSystems;
/*
An arm with two grip mechanisms and rotatable on a motor to almost 225 degrees.
Arm is controlled by trigger button and has fast motion from parked position to
position for dropping ring in low goal, and from there slow motion to position
for dropping ring on wobble goal, picking wobble goal position and picking ring
from floor position
*/

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HzArm {

    public DcMotor armMotor;
    //Gobilda 5202 Series Yellow Jacket Planetary Gear Motor (26.9:1 Ratio, 223 RPM, 3.3 - 5V Encoder)
    //Encoder count : 753.2 - mounted on a 2:1 gear ratio

    public Servo armGripServo;

    public enum ARM_POSITION {
        PARKED,
        HOLD_UP_WOBBLE_RING,
        DROP_WOBBLE_RING,
        PICK_WOBBLE,
        PICK_RING
    }

    public static int ARM_PARKED_POSITION_COUNT = 0;
    public static int ARM_HOLD_UP_WOBBLE_RING_POSITION_COUNT = -250;//-350 ;
    public static int ARM_DROP_WOBBLE_RING_POSITION_COUNT = -500 ;
    public static int ARM_PICK_WOBBLE_POSITION_COUNT = -700 ;
    public static int ARM_PICK_RING_POSITION_COUNT = -875 ;

    public static double POWER_NO_WOBBLEGOAL = 0.3;
    public static double POWER_WITH_WOBBLEGOAL = 0.6;

    public ARM_POSITION currentArmPosition = ARM_POSITION.PARKED;
    public ARM_POSITION previousArmPosition = ARM_POSITION.PARKED;;

    public enum GRIP_SERVO_STATE {
        OPENED,
        CLOSED
    };

    public static final double GRIP_OPEN = 1.0, GRIP_CLOSE = 0.54;

    public GRIP_SERVO_STATE gripServoState = GRIP_SERVO_STATE.OPENED ;

    public HzArm(HardwareMap hardwareMap) {
        armMotor = hardwareMap.dcMotor.get("arm_rotate");
        armGripServo = hardwareMap.servo.get("arm_grip");
    }

    LinearOpMode opModepassed;

    public void initArm(/*LinearOpMode opModepassed1*/){
        //this.opModepassed = opModepassed1;
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        resetArm();
        moveArmParkedPosition();
        turnArmBrakeModeOn();
        initGrip();
    }

    public void resetArm(){
        DcMotor.RunMode runMode = armMotor.getMode();
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(runMode);
    }

    /**
     * Method to set Arm brake mode to ON when Zero (0.0) power is applied. <BR>
     * To be used when arm is above groundlevel
     * setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
     */
    public void turnArmBrakeModeOn(){
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //armMotor.setPower(0.0);
    }

    /**
     * Method to set Arm brake mode to OFF when Zero (0.0) power is applied. <BR>
     * To be used when arm is on groundlevel or blockLevel[0]
     * setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
     */
    public void turnArmBrakeModeOff(){
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //armMotor.setPower(0.0);
    }

    /**
     * Method to run motor to set to the set position
     */


    public boolean runArmToLevelState = false;
    public double motorPowerToRun = POWER_NO_WOBBLEGOAL;

    public void runArmToLevel(double power){
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int sign = armMotor.getCurrentPosition() < armMotor.getTargetPosition() ? 1 : -1;
        if (runArmToLevelState = true && (sign*(armMotor.getCurrentPosition() - armMotor.getTargetPosition()) > 0 )) {
            armMotor.setPower(0.0);
            runArmToLevelState = false;
            return;
        } else {
            if (!armMotor.isBusy()) {
                armMotor.setPower(sign * power);
            }
        }
    }

    public void moveArmParkedPosition() {
        turnArmBrakeModeOff();
        armMotor.setTargetPosition(ARM_PARKED_POSITION_COUNT);
        motorPowerToRun = POWER_NO_WOBBLEGOAL;
        runArmToLevelState = true;
        currentArmPosition = ARM_POSITION.PARKED;
    }

    public void moveArmHoldUpWobbleRingPosition() {
        turnArmBrakeModeOn();
        armMotor.setTargetPosition(ARM_HOLD_UP_WOBBLE_RING_POSITION_COUNT);
        motorPowerToRun = POWER_WITH_WOBBLEGOAL;
        runArmToLevelState = true;
        currentArmPosition = ARM_POSITION.HOLD_UP_WOBBLE_RING;
    }


    public void moveArmDropWobbleRingPosition() {
        turnArmBrakeModeOn();
        armMotor.setTargetPosition(ARM_DROP_WOBBLE_RING_POSITION_COUNT);
        motorPowerToRun = POWER_NO_WOBBLEGOAL;
        runArmToLevelState = true;
        currentArmPosition = ARM_POSITION.DROP_WOBBLE_RING;
    }

    public void moveArmPickWobblePosition() {
        turnArmBrakeModeOn();
        armMotor.setTargetPosition(ARM_PICK_WOBBLE_POSITION_COUNT);
        motorPowerToRun = POWER_NO_WOBBLEGOAL;
        runArmToLevelState = true;
        currentArmPosition = ARM_POSITION.PICK_WOBBLE;
    }

    public void moveArmPickRingPosition() {
        turnArmBrakeModeOn();
        armMotor.setTargetPosition(ARM_PICK_RING_POSITION_COUNT);
        motorPowerToRun = POWER_NO_WOBBLEGOAL;
        runArmToLevelState = true;
        currentArmPosition = ARM_POSITION.PICK_RING;
    }

    public void moveArmByTrigger() {
        if ((currentArmPosition== ARM_POSITION.PARKED)) {
            previousArmPosition = currentArmPosition;
            moveArmPickWobblePosition();
            openGrip();
            return;
        }

        //To go to ring position, grip should be open and triggered again from pick position
        if ((currentArmPosition== ARM_POSITION.PICK_WOBBLE)  &&
                (getGripServoState() == GRIP_SERVO_STATE.OPENED)) {
            previousArmPosition = currentArmPosition;
            moveArmPickRingPosition();
            return;
        }

        if ((currentArmPosition== ARM_POSITION.PICK_RING)  &&
                (getGripServoState() == GRIP_SERVO_STATE.OPENED)) {
            previousArmPosition = currentArmPosition;
            closeGrip();
            moveArmParkedPosition();
            return;
        }

        // After closing grip in pick wobble or ring position - assumes holding of wobble goal is done
        if ((currentArmPosition== ARM_POSITION.PICK_WOBBLE)  &&
                (previousArmPosition == ARM_POSITION.PARKED) &&
                (getGripServoState() == GRIP_SERVO_STATE.CLOSED)) {
            previousArmPosition = currentArmPosition;
            moveArmHoldUpWobbleRingPosition();
            return;
        }

        // After closing grip in pick wobble or ring position - assumes holding of wobble goal is done
        if ((currentArmPosition== ARM_POSITION.PICK_RING)  &&
                (getGripServoState() == GRIP_SERVO_STATE.CLOSED)) {
            previousArmPosition = currentArmPosition;
            moveArmHoldUpWobbleRingPosition();
            return;
        }

        if ((currentArmPosition== ARM_POSITION.HOLD_UP_WOBBLE_RING)  &&
                (getGripServoState() == GRIP_SERVO_STATE.CLOSED)) {
            previousArmPosition = currentArmPosition;
            moveArmDropWobbleRingPosition();
            return;
        }

        if ((currentArmPosition== ARM_POSITION.DROP_WOBBLE_RING)  &&
                (getGripServoState() == GRIP_SERVO_STATE.OPENED)) {
            previousArmPosition = currentArmPosition;
            closeGrip();
            moveArmParkedPosition();
            return;
        }

        if ((currentArmPosition== ARM_POSITION.DROP_WOBBLE_RING)  &&
                (getGripServoState() == GRIP_SERVO_STATE.CLOSED)) {
            previousArmPosition = currentArmPosition;
            moveArmHoldUpWobbleRingPosition();
            return;
        }

    }



    //**** Grip Methods ****
    public void initGrip() {
        // On init close grip - In Autonomous mode, this will be used by drive to make robot hold the wobble goal
        armGripServo.setPosition(GRIP_CLOSE);
        gripServoState = GRIP_SERVO_STATE.CLOSED;
    }

    public void openGrip() {
        if((currentArmPosition!= ARM_POSITION.PARKED) &&
                (currentArmPosition!= ARM_POSITION.HOLD_UP_WOBBLE_RING))  {
            armGripServo.setPosition(GRIP_OPEN);
            gripServoState = GRIP_SERVO_STATE.OPENED;
        }
    }

    public void closeGrip() {
        armGripServo.setPosition(GRIP_CLOSE);
        gripServoState = GRIP_SERVO_STATE.CLOSED;
    }

    // grips ring with hand of the arm by running armRingGripServo
    public ARM_POSITION getCurrentArmPosition() {
        return currentArmPosition;
    }

    public GRIP_SERVO_STATE getGripServoState() {
        return gripServoState;
    }
}