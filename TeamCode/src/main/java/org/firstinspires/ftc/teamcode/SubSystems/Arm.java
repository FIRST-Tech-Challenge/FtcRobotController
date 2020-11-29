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
import com.qualcomm.robotcore.util.ElapsedTime;

public class Arm {

    public DcMotor armMotor;
    //Gobilda 5202 Series Yellow Jacket Planetary Gear Motor (26.9:1 Ratio, 223 RPM, 3.3 - 5V Encoder)
    //Encoder count : 753.2 - mounted on a 2:1 gear ratio

    public Servo armGripServo;

    public enum ARM_POSITIONS {
        PARKED,
        DROP_WOBBLE_GOAL,
        HOLD_WOBBLE_GOAL,
        PICK_RING
    }

    public static int ARM_PARKED_POSITION_COUNT = 0;
    public static int ARM_DROP_WOBBLE_GOAL_POSITION_COUNT = -450 ;
    public static int ARM_HOLD_WOBBLE_GOAL_POSITION_COUNT = -700 ;
    public static int ARM_RING_POSITION_COUNT = -875 ;

    public ARM_POSITIONS currentArmPosition = ARM_POSITIONS.PARKED;

    public enum GRIP_SERVO_STATE {
        OPENED,
        CLOSED
    };

    public static final double GRIP_OPEN = 1.0, GRIP_CLOSE = 0.5;

    public GRIP_SERVO_STATE gripServoState = GRIP_SERVO_STATE.OPENED ;

    public Arm(HardwareMap hardwareMap) {
        armMotor = hardwareMap.dcMotor.get("arm_rotate");
        armGripServo = hardwareMap.servo.get("arm_grip");
    }

    LinearOpMode opModepassed;

    public void initArm(/*LinearOpMode opModepassed1*/){
        //this.opModepassed = opModepassed1;
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        armMotor.setPower(0.0);
    }

    /**
     * Method to set Arm brake mode to OFF when Zero (0.0) power is applied. <BR>
     * To be used when arm is on groundlevel or blockLevel[0]
     * setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
     */
    public void turnArmBrakeModeOff(){
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        armMotor.setPower(0.0);
    }


    /**
     * Method to run motor to set to the set position
     */
    public void runArmToLevel() {
        //armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //Turn Motors on
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        timer.reset();
        int sign = armMotor.getCurrentPosition() < armMotor.getTargetPosition() ? 1 : -1;
        armMotor.setPower(sign * 0.2);
        while((sign*(armMotor.getCurrentPosition() - armMotor.getTargetPosition()) < 0 ) && timer.time() < 4){
            //opModepassed.telemetry.addData("armMotor.getTargetPosition()", armMotor.getTargetPosition());
            //opModepassed.telemetry.addData("armMotor.getCurrentPosition()", armMotor.getCurrentPosition());
            //opModepassed.telemetry.update();
        }
    }

    public void moveArmParkedPosition() {
        armMotor.setTargetPosition(ARM_PARKED_POSITION_COUNT);
        runArmToLevel();
        turnArmBrakeModeOn();
        currentArmPosition = ARM_POSITIONS.PARKED;
    }

    public void moveArmDropWobbleGoalPosition() {
        armMotor.setTargetPosition(ARM_DROP_WOBBLE_GOAL_POSITION_COUNT);
        runArmToLevel();
        turnArmBrakeModeOn();
        currentArmPosition = ARM_POSITIONS.DROP_WOBBLE_GOAL;
    }

    public void moveArmHoldWobbleRingPosition() {
        armMotor.setTargetPosition(ARM_HOLD_WOBBLE_GOAL_POSITION_COUNT);
        runArmToLevel();
        turnArmBrakeModeOn();
        currentArmPosition = ARM_POSITIONS.HOLD_WOBBLE_GOAL;
    }

    public void moveArmRingPosition() {
        armMotor.setTargetPosition(ARM_RING_POSITION_COUNT);
        runArmToLevel();
        turnArmBrakeModeOn();
        currentArmPosition = ARM_POSITIONS.PICK_RING;
    }

    public void moveArmByTrigger(double triggerPosition) {
        if ((triggerPosition < 0.1) && (currentArmPosition!= ARM_POSITIONS.PARKED)) {
            moveArmParkedPosition();
        } else if ((triggerPosition > 0.1) && (triggerPosition < 0.5) &&
                (currentArmPosition!= ARM_POSITIONS.DROP_WOBBLE_GOAL)) {
            moveArmDropWobbleGoalPosition();
        } else if ((triggerPosition > 0.5) && (triggerPosition < 0.9) &&
                (currentArmPosition!= ARM_POSITIONS.HOLD_WOBBLE_GOAL)) {
            moveArmHoldWobbleRingPosition();
        } else if ((triggerPosition > 0.9) &&
                (currentArmPosition!= ARM_POSITIONS.PICK_RING)) {
            moveArmRingPosition();
        }

        /*opModepassed.telemetry.addData("initialArmPositionCount", initialArmPositionCount);
        opModepassed.telemetry.addData("armMotor.getCurrentPosition()", armMotor.getCurrentPosition());
        opModepassed.telemetry.addData("triggerPosition", triggerPosition);
        opModepassed.telemetry.addData("armGripServo.getCurrentPosition()", armGripServo.getPosition());
        opModepassed.telemetry.update();

         */
    }

    //**** Grip Methods ****
    public void initGrip() {
        // On init close grip - In Autonomous mode, this will be used by drive to make robot hold the wobble goal
        armGripServo.setPosition(GRIP_CLOSE);

        // AMJAD : Changed hardware design to use on one servo for both wobble goal and grip
        //armRingGripServo.setPosition(GRIP_CLOSE);
        gripServoState = GRIP_SERVO_STATE.CLOSED;
    }

    public void openGrip() {
        armGripServo.setPosition(GRIP_OPEN);
        gripServoState = GRIP_SERVO_STATE.OPENED;
    }

    public void closeGrip() {
        armGripServo.setPosition(GRIP_CLOSE);
        gripServoState = GRIP_SERVO_STATE.CLOSED;
    }

    // grips ring with hand of the arm by running armRingGripServo
    public ARM_POSITIONS getCurrentArmPosition() {
        return currentArmPosition;
    }

    public GRIP_SERVO_STATE getGripServoState() {
        return gripServoState;
    }
}