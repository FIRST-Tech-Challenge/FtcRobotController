package org.firstinspires.ftc.teamcode.SubSystems;
/**
 * An arm with two grip mechanisms and rotatable on a motor to almost 225 degrees.
 * Arm is controlled by trigger button and has fast motion from parked position to
 * position for dropping ring in low goal, and from there slow motion to position
 * for dropping ring on wobble goal, picking wobble goal position and picking ring
 * from floor position
*/

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HzArm {

    public DcMotorEx armMotor;
    //Gobilda 5202 Series Yellow Jacket Planetary Gear Motor (26.9:1 Ratio, 223 RPM, 3.3 - 5V Encoder)
    //Encoder count : 753.2 - mounted on a 2:1 gear ratio

    public Servo armGripServo;

    public enum ARM_POSITION {
        PARKED,
        HOLD_UP_WOBBLE_RING,
        DROP_WOBBLE_RING,
        DROP_WOBBLE_AUTONOMOUS,
        PICK_WOBBLE,
        PICK_RING
    }

    public static int baselineEncoderCount = 0;
    public static int ARM_PARKED_POSITION_COUNT = 0;
    public static int ARM_HOLD_UP_WOBBLE_RING_POSITION_COUNT = -250;
    public static int ARM_DROP_WOBBLE_RING_POSITION_COUNT = -500 ;
    public static int ARM_DROP_WOBBLE_AUTONOMOUS_POSITION = -700;
    public static int ARM_PICK_WOBBLE_POSITION_COUNT = -725 ;
    public static int ARM_PICK_RING_POSITION_COUNT = -900 ;

    public static double POWER_NO_WOBBLEGOAL = 0.6;
    public static double POWER_WITH_WOBBLEGOAL = 0.35;

    public ARM_POSITION currentArmPosition = ARM_POSITION.PARKED;
    public ARM_POSITION previousArmPosition = ARM_POSITION.PARKED;;

    public enum GRIP_SERVO_STATE {
        OPENED,
        CLOSED
    };

    public static final double GRIP_OPEN = 1.0, GRIP_CLOSE = 0.53;

    public GRIP_SERVO_STATE gripServoState = GRIP_SERVO_STATE.OPENED ;

    public HzArm(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotorEx.class, "arm_rotate");
        armGripServo = hardwareMap.servo.get("arm_grip");
    }

    LinearOpMode opModepassed;

    /**
     * Initialization for the Arm
     */
    public void initArm(){

        armMotor.setPositionPIDFCoefficients(5.0);
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        resetArm();
        moveArmParkedPosition();
        turnArmBrakeModeOn();
        initGrip();
    }

    /**
     * Reset Arm Encoder
     */
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
    }

    /**
     * Method to set Arm brake mode to OFF when Zero (0.0) power is applied. <BR>
     * To be used when arm is on groundlevel or blockLevel[0]
     * setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
     */
    public void turnArmBrakeModeOff(){
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public boolean runArmToLevelState = false;
    public double motorPowerToRun = POWER_NO_WOBBLEGOAL;

    /**
     * Method to run motor to set to the set position
     */
    public void runArmToLevel(double power){
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (runArmToLevelState == true || armMotor.isBusy() == true){
            armMotor.setPower(power);
            runArmToLevelState = false;
        } else {
            armMotor.setPower(0.0);
        }
    }

    /**
     * Move Arm to Park Position
     */
    public void moveArmParkedPosition() {
        turnArmBrakeModeOff();
        armMotor.setTargetPosition(ARM_PARKED_POSITION_COUNT + baselineEncoderCount);
        motorPowerToRun = POWER_NO_WOBBLEGOAL;
        runArmToLevelState = true;
        currentArmPosition = ARM_POSITION.PARKED;
    }

    /**
     * Move Arm to Holding up position of Wobble and Ring in TeleOp Mode
     */
    public void moveArmHoldUpWobbleRingPosition() {
        turnArmBrakeModeOn();
        armMotor.setTargetPosition(ARM_HOLD_UP_WOBBLE_RING_POSITION_COUNT + baselineEncoderCount);
        motorPowerToRun = POWER_WITH_WOBBLEGOAL;
        runArmToLevelState = true;
        currentArmPosition = ARM_POSITION.HOLD_UP_WOBBLE_RING;
    }


    /**
     * Move Arm to Drop Wobble goal and Ring in TeleOp Mode
     */
    public void moveArmDropWobbleRingPosition() {
        turnArmBrakeModeOn();
        armMotor.setTargetPosition(ARM_DROP_WOBBLE_RING_POSITION_COUNT + baselineEncoderCount);
        motorPowerToRun = POWER_NO_WOBBLEGOAL;
        runArmToLevelState = true;
        currentArmPosition = ARM_POSITION.DROP_WOBBLE_RING;
    }

    /**
     * Move Arm to Drop Wobble Goal in autonomous
     */
    public void moveArmDropWobbleAutonomousPosition() {
        turnArmBrakeModeOn();
        armMotor.setTargetPosition(ARM_DROP_WOBBLE_AUTONOMOUS_POSITION + baselineEncoderCount);
        motorPowerToRun = POWER_WITH_WOBBLEGOAL;
        runArmToLevelState = true;
        currentArmPosition = ARM_POSITION.DROP_WOBBLE_AUTONOMOUS;
    }


    /**
     * Move arm to Pick Wobble Position
     */
    public void moveArmPickWobblePosition() {
        turnArmBrakeModeOn();
        armMotor.setTargetPosition(ARM_PICK_WOBBLE_POSITION_COUNT + baselineEncoderCount);
        motorPowerToRun = POWER_NO_WOBBLEGOAL;
        runArmToLevelState = true;
        currentArmPosition = ARM_POSITION.PICK_WOBBLE;
    }

    /**
     * Move Arm to Pick Ring Position
     */
    public void moveArmPickRingPosition() {
        turnArmBrakeModeOn();
        armMotor.setTargetPosition(ARM_PICK_RING_POSITION_COUNT + baselineEncoderCount);
        motorPowerToRun = POWER_NO_WOBBLEGOAL;
        runArmToLevelState = true;
        currentArmPosition = ARM_POSITION.PICK_RING;
    }

    /**
     * Method for moving arm based on trigger inputs.
     * Depending on the grip condition in the pick wobble goal position or the pic ring position
     * the next motion is determined
     */
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

    /**
     * Open Arm Grip
     */
    public void openGrip() {
        if((currentArmPosition!= ARM_POSITION.PARKED) &&
                (currentArmPosition!= ARM_POSITION.HOLD_UP_WOBBLE_RING))  {
            armGripServo.setPosition(GRIP_OPEN);
            gripServoState = GRIP_SERVO_STATE.OPENED;
        }
    }

    /**
     * Close Arm grip)
     */
    public void closeGrip() {
        armGripServo.setPosition(GRIP_CLOSE);
        gripServoState = GRIP_SERVO_STATE.CLOSED;
    }


    /**
     * Return the current Arm Position
     */
    public ARM_POSITION getCurrentArmPosition() {
        return currentArmPosition;
    }

    /**
     * Return the current Grip position
     */
    public GRIP_SERVO_STATE getGripServoState() {
        return gripServoState;
    }
}