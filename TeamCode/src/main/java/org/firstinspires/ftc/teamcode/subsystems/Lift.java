package org.firstinspires.ftc.teamcode.subsystems;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.utils.PIDController;

public class Lift {

    //Adjustable Constants
    public int LIFT_SPEED = 50; //ticks per call
    public double POWER = 1; //Max Power


    //Lift Positions and possible States

    public enum LiftStates{
        ZERO,
        HOVER,
        CLIMB,
        SPECIMEN_SCORE,
        HIGH_BAR,
        MAX_HEIGHT
    }
    private LiftStates currentState;
    public int ZERO = 0; //Ticks //SAMPLE INTAKE
    public int HOVER = 220; //Ticks //Sample HOVER

    public int CLIMB_HEIGHT = 1600;
    public int HIGH_BAR = 1700; //Ticks //SPECIMIN DESPOSIT
    public int SPECIMEN_SCORE = 1200; //Ticks //SPECIMIN SCORE
    public int MAX_HEIGHT = 3050; //Ticks //SAMPLE DEPOSIT
    //Was planning to tune weight, but just stuck with the mass of 1 kg
    public double calculatedWeight = 9.8;

    //Internal variables
    private final DcMotorEx liftLeft, liftRight;

    private int targetPosition;

    private final Encoder encoder;
    private final PIDController pid;


    //-----------------------------------------------------------------------------------------
    //----------------------------------Initialization-----------------------------------------
    //-----------------------------------------------------------------------------------------
    /**
     * Quick Constructor for the Lift Subsystem Class
     * @param hw [HardwareMap] Hardware map necessary to initialize the motors.
     *///
    public Lift(HardwareMap hw){
        this(hw, "liftLeft", "liftRight", "FRM");
    }

    /**
     * Primary constructor for the Lift Subsystem Class
     * @param hw [HardwareMap] Hardware map necessary to initialize motors.
     * @param nameLeft [String] Name of the left motor assigned in the configuration.
     * @param nameRight [String] Name of the right motor assigned in the configuration.
     */
    public Lift(HardwareMap hw, String nameLeft, String nameRight, String nameEncoder){
        //Initialize motors
        this.liftLeft = hw.get(DcMotorEx.class, nameLeft);
        this.liftRight = hw.get(DcMotorEx.class, nameRight);
        encoder = new OverflowEncoder(new RawEncoder(hw.get(DcMotorEx.class, nameEncoder)));
        //Reverse one of the motors
        this.liftRight.setDirection(DcMotorSimple.Direction.REVERSE);

        currentState = LiftStates.ZERO;


        pid = new PIDController(0.009,0,0.0002,0.02);
        pid.setTarget(getPosition());
    }



    //-----------------------------------------------------------------------------------------
    //----------------------------------Go To Positions----------------------------------------
    //-----------------------------------------------------------------------------------------

    /**
     * Sets the motors' target position to [ZERO] (lowest possible position)
     */
    public void goToZero(){
        currentState = LiftStates.ZERO;
        this.targetPosition = ZERO;
        pid.setTarget(this.targetPosition);
    }

    /**
     * Sets the motors' target position to [MAX_HEIGHT] (highest possible position)
     */
    public void goToTopBucket(){
        currentState = LiftStates.MAX_HEIGHT;
        this.targetPosition = MAX_HEIGHT;
        pid.setTarget(this.targetPosition);
    }

    /**
     * Sets the motors' target position to [SPECIMIN_SCORE]
     */
    public void goToSpecimenScore(){
        currentState = LiftStates.SPECIMEN_SCORE;
        this.targetPosition = SPECIMEN_SCORE;
        pid.setTarget(this.targetPosition);
    }

    /**
     * Sets the motors' target position to [HIGH_BAR]
     */
    public void goToHighBar(){
        currentState = LiftStates.HIGH_BAR;
        this.targetPosition = HIGH_BAR;
        pid.setTarget(this.targetPosition);
    }

    /**
     * Sets the motors' target position to [HOVER]
     */
    public void goToSubHover(){
        currentState = LiftStates.HOVER;
        this.targetPosition = HOVER;
        pid.setTarget(this.targetPosition);
    }

    /**
     * Sets the motors' target position to [CLIMB]
     */
    public void goToClimb(){
        currentState = LiftStates.CLIMB;
        this.targetPosition = CLIMB_HEIGHT;
        pid.setTarget(this.targetPosition);
    }

    /**
     * This will cause the lift to travel to the given LiftState.
     * @param state [LiftStates] - Sets the lift to go to the given Lift Position State.
     */
    public void goToPosition(LiftStates state){
        switch (state){
            case ZERO:
                goToZero();
                break;

            case HOVER:
                goToSubHover();
                break;

            case HIGH_BAR:
                goToHighBar();
                break;

            case CLIMB:
                goToClimb();
                break;

            case MAX_HEIGHT:
                goToTopBucket();
                break;

            case SPECIMEN_SCORE:
                goToSpecimenScore();
                break;

            default:
                //Do Nothing
                break;
        }
    }
    //------------------------------------------------------------------------------------------
    //----------------------------------Getter Functions----------------------------------------
    //------------------------------------------------------------------------------------------
    /**
     * Tester function to track current height.
     * @return [int] Returns current position of the left motor in ticks.
     */
    public int getPosition(){
        return encoder.getPositionAndVelocity().position;
    }
    public int getVelocity() {return encoder.getPositionAndVelocity().velocity;}

    public int getTargetPosition(){
        return targetPosition;
    }
    public LiftStates getState(){
        return currentState;
    }

    public double getForwardFeedValue(){
        return calculatedWeight;
    }

    public PIDController getPid(){
        return pid;
    }



    //-----------------------------------------------------------------------------------------
    //----------------------------------Tune/Tweak Functions-----------------------------------
    //-----------------------------------------------------------------------------------------
    /**
     * Increases/decreases a PID tuning value by a set amount
     * @param Kp [double] Increment to increase Kp by
     * @param Ki [double] Increment to increase Ki by
     * @param Kd [double] Increment to increase Kd by
     */
    public void adjustPID(double Kp, double Ki, double Kd, double Kf){
//        double[] k = pid.getPIDValues();
        pid.setKp(Kp);
        pid.setKi(Ki);
        pid.setKd(Kd);
        pid.setKf(Kf);
    }


    /**
     * Manual tester to adjust the height of the lift.
     * @param power [double] Can be paired with a joystick or trigger to have a dynamic speed to raise the lift.
     * @return [int] Returns the new target position of the lift in ticks.
     */
    public int setPosition(double power){
        targetPosition += (power * LIFT_SPEED);
        //Toggle this once MAX_HEIGHT has been configured
//        targetPosition = clamp(targetPosition, MAX_HEIGHT, 0);
        pid.setTarget(targetPosition);
        return targetPosition;
    }

    /**
     * Updates PID loop/motor power.
     * Ensure this is called when using lift, otherwise nothing will happen.
     */
    public double update(){
        double power = pid.calculate(getPosition(), getForwardFeedValue());
        liftLeft.setPower(power);
        liftRight.setPower(power);
        return power;
    }

    //-----------------------------------------------------------------------------------------
    //----------------------------------Helper Functions---------------------------------------
    //-----------------------------------------------------------------------------------------
    @SuppressLint("DefaultLocale")
    @Override
    public String toString(){
        return String.format(
                "Arm current position: %d\n" +
                        "Arm target position: %d\n" +
                        "Arm PID Data: \n%s",
                getPosition(),
                targetPosition,
                pid.toString());
    }

    @SuppressLint("DefaultLocale")
    public String getArmCurrent(){
        return String.format(
                "Left Arm Current: %f\n" +
                        "Right Arm Current: %f",
                liftLeft.getCurrent(CurrentUnit.AMPS),
                liftRight.getCurrent(CurrentUnit.AMPS)
        );
    }

    private double clamp(double value, double max, double min){
        return Math.max( min , Math.min( max , value));
    }

    //-----------------------------------------------------------------------------------------
    //----------------------------------Autonomous Actions-------------------------------------
    //-----------------------------------------------------------------------------------------
    public Action liftPID(){
        return new LiftPID();
    }


    public class LiftPID implements  Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            update();
            return true;
        }
    }


}
