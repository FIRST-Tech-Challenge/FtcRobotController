package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Definition of Subsystem Class <BR>
 *
 * Example : Intake consists of system provided intake controls and adds functionality to the selection made on intake. <BR>
 *
 * The states are as followed: <BR>
 *     <emsp>SUBSYSTEM1_SERVO_LEVEL1 for one state - example if intake motor is running, stopped, or reversing </emsp> <BR>
 *     <emsp>SUBSYSTEM1_SERVO_LEVEL2 for another state  = example if the intake is on or off </emsp> <BR>
 *
 * The functions are as followed: Example assumes a motor like an intake <BR>
 *     <emsp>runSubsystem1Motor checks if the motor is not running and runs the intake </emsp> <BR>
 *     <emsp>stopSubsystem1Motor checks if the intake has stopped and if its not, it sets the intake power to 0
 *     and sets subsystem1MotorState to SUBSYSTEM1_SERVO_LEVEL1.STOPPED </emsp> <BR>
 *     <emsp> startReverseSubsystem1Motor checks if the motor is not reversing, and sets the  motor to FORWARD, then also
 *     sets intake motor state to REVERSING</emsp> <BR>
 */
public class MajorArm {

    public Servo majorClawServo;

    public enum MAJOR_CLAW_STATE {
        CLAW_OPEN,
        CLAW_CLOSED,
    }

    public DcMotorEx majorArmMotor;

    public enum ARM_POSITION {
        PICKUP,
        LEVEL_1,
        LEVEL_2,
        LEVEL_3,
        CAPSTONE,
        PARKED,
    }

    public MajorArm(HardwareMap hardwareMap) {
        majorArmMotor = hardwareMap.get(DcMotorEx.class, "arm_rotate");
        majorArmGrip = hardwareMap.servo.get("arm_grip");
    }

    public static int baselineEncoderCount = 0;
    public static int CLAW_OPEN = __;
    public static int CLAW_CLOSED = __;
    public MAJOR_CLAW_STATE majorClawState = MAJOR_CLAW_STATE.OPENED;
    public static int PICKUP = __;
    public static int LEVEL_1 = __;
    public static int LEVEL_2 = __;
    public static int LEVEL_3 = __;
    public static int CAPSTONE = __;
    public static int PARKED = __;
    public ARM_POSITION currentArmPosition = ARM_POSITION.PARKED;
    public ARM_POSITION previousArmPosition = ARM_POSITION.PARKED;

    public void initMajorArm(){
        turnArmBrakeModeOn();
        majorClawServo.setPosition(CLAW.CLOSED);
        majorClawState = MAJOR_CLAW_STATE.CLOSED;
    }

    public MAJOR_CLAW_STATE getMajorClawState() {
        return majorClawState;
    }
    public ARM_POSITION getArmPosition() {
        return currentArmPosition;
    }
    public void turnArmBrakeModeOn(){
        majorArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void turnArmBrakeModeOff(){
        majorArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void changeGripState() {
        if ((majorClawState == MAJOR_CLAW.OPEN)) {
            majorClawServo.setPosition(CLAW.CLOSED);
            majorClawState = MAJOR_CLAW_STATE.CLOSED;
        } else if ((majorClawState == MAJOR_CLAW.CLOSED)) {
            majorClawServo.setPosition(CLAW.OPEN);
            majorClawState = MAJOR_CLAW_STATE.OPEN;
        }
    }
//change the level of the Arm to Capstone
    public void moveArmCapstonePosition() {
        turnArmBrakeModeOff();
        majorArmMotor.setTargetPosition(CAPSTONE + baselineEncoderCount);
        currentArmPosition = ARM_POSITION.CAPSTONE;
    }
//change the level of the Arm to Pickup
    public void moveArmPickupPosition() {
        turnArmBrakeModeOff();
        majorArmMotor.setTargetPosition(PICKUP + EncoderCount);
        currentArmPosition = ARM_POSITION.PICKUP;
    }
//change the level of the arm to Level One
    public void moveArmLevelOnePosition() {
        turnArmBrakeModeOff();
        majorArmMotor.setTargetPosition(LEVEL_1 + baselineEncoderCount);
        currentArmPosition = ARM_POSITION.LEVEL_1;
    }
//change the level of the arm to Level Two
    public void moveArmLevelTwoPosition() {
        turnArmBrakeModeOff();
        majorArmMotor.setTargetPosition(LEVEL_2 + baselineEncoderCount);
        currentArmPosition = ARM_POSITION.LEVEL_2;
    }
//change the level of the arm to level three
    public void moveArmLevelThreePosition() {
        turnArmBrakeModeOff();
        majorArmMotor.setTargetPosition(LEVEL_3 + baselineEncoderCount);
        currentArmPosition = ARM_POSITION.LEVEL_3;
    }
//change the level of the arm to the parking
    public void moveArmParkingPosition() {
        turnArmBrakeModeOff();
        majorArmMotor.setTargetPosition(PARKING + baselineEncoderCount);
        currentArmPosition = ARM_POSITION.PARKING;
    }
    //change the level of the Arm to Level 1
    public moveArmUpOne() {
        if ((currentArmPosition == ARM_POSITION.PICKUP)) {
            previousArmPosition = currentArmPosition;
            moveArmLevelOnePosition();
            return;
        }
        if ((currentArmPosition == ARM_POSITION.LEVEL_1)) {
            previousArmPosition = currentArmPosition;
            moveArmLevelTwoPosition();
            return;
        }
        if ((currentArmPosition == ARM_POSITION.LEVEL_2)) {
            previousArmPosition = currentArmPosition;
            moveArmLevelThreePosition();
            return;
        }
        if ((currentArmPosition == ARM_POSITION.LEVEL_3)) {
            previousArmPosition = currentArmPosition;
            moveArmLevelCapstonePosition();
            return;
        }
        if ((currentArmPosition == ARM_POSITION.LEVEL_CAPSTONE)) {
            previousArmPosition = currentArmPosition;
            moveArmLevelParkedPosition();
            return;
        }
    }

    //change the level of the Arm by one Down
    public moveArmDownOne() {
        if ((currentArmPosition == ARM_POSITION.PARKING)) {
            previousArmPosition = currentArmPosition;
            moveArmLevelCapstonePosition();
            return;
        }
        if ((currentArmPosition == ARM_POSITION.CAPSTONE)) {
            previousArmPosition = currentArmPosition;
            moveArmLevelThreePosition();
            return;
        }
        if ((currentArmPosition == ARM_POSITION.LEVEL_3)){
            previousArmPosition = currentArmPosition;
            moveArmLevelTwoPosition();
            return;
        }
        if ((currentArmPosition == ARM_POSITION.LEVEL_2)) {
            previousArmPosition = currentArmPosition;
            moveArmLevelLevelOnePosition();
            return;
        }
        if ((currentArmPosition == ARM_POSITION.LEVEL_ONE)) {
            previousArmPosition = currentArmPosition;
            moveArmLevelPickupPosition();
            return;
        }
    }
}