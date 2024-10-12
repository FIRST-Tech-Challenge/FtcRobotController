package org.firstinspires.ftc.teamcode.subsystems;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.PIDController;

public class Lift {

    /*TODO LIST:
        Identify [Height Limits] of Lift
        Identify [Position Heights] of Lift
        Tune PID Parameters accordingly
    */
    //Adjustable Constants
    public int LIFT_SPEED = 20; //ticks per call
    public double POWER = 1; //Max Power
    public int LOW_HEIGHT = 0; //Ticks
    public int MEDIUM_HEIGHT = 1000; //Ticks
    public int HIGH_HEIGHT = 2000; //Ticks

    public int MAX_HEIGHT = 2000;

    public double GRAVITY = 9.8; // N/kgs
    public double ARM_WEIGHT = 1;//kgs
    public double SLIDE_WEIGHT = 0.3; //kgs
    public double TICKS_PER_SLIDE = 500; //ticks - Figure out later
    public double calculatedWeight = GRAVITY * ARM_WEIGHT;

    //Internal variables
    private DcMotor liftLeft, liftRight;
    private int targetPosition;

    private Encoder encoder;
    private PIDController pid;


    /**
     * Quick Constructor for the Lift Subsystem Class
     * @param hw [HardwareMap] Hardware map necessary to initialize the motors.
     *///TODO name encoder according to wiring described by Ryan
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
        this.liftLeft = hw.get(DcMotor.class, nameLeft);
        this.liftRight = hw.get(DcMotor.class, nameRight);
        encoder = new OverflowEncoder(new RawEncoder(hw.get(DcMotorEx.class, nameEncoder)));
        //Reverse one of the motors
        this.liftRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //Ensures motor encoders are reset

        pid = new PIDController(0,0,0,0);
        pid.setTarget(getPosition());
    }

    /**
     * Manual tester to adjust the height of the lift.
     * @param power [double] Can be paired with a joystick or trigger to have a dynamic speed to raise the lift.
     * @return [int] Returns the new target position of the lift in ticks.
     */
    public int moveLift(double power){
        targetPosition += (power * LIFT_SPEED);
        //Toggle this once MAX_HEIGHT has been configured
//        targetPosition = clamp(targetPosition, MAX_HEIGHT, 0);
        pid.setTarget(targetPosition);
        return targetPosition;
    }


    private double clamp(double value, double max, double min){
        return Math.max( min , Math.min( max , value));
    }

    /**
     * Tester function to track current height.
     * @return [int] Returns current position of the left motor in ticks.
     */
    public int getPosition(){
        return encoder.getPositionAndVelocity().position;
    }


    /**
     * Sets the motors' target position to [LOW_HEIGHT]
     */
    public void goToLow(){
        pid.setTarget(LOW_HEIGHT);
    }
    /**
     * Sets the motors' target position to [MEDIUM_HEIGHT]
     */
    public void goToMedium(){
        pid.setTarget(MEDIUM_HEIGHT);
    }

    /**
     * Sets the motors' target position to [HIGH_HEIGHT]
     */
    public void goToHigh(){
        pid.setTarget(HIGH_HEIGHT);
    }

    public double getForwardFeedValue(){
        return calculatedWeight;
    }
    public void recalculateWeight(){
        calculatedWeight = GRAVITY * (ARM_WEIGHT + (int) (pid.getTarget()/TICKS_PER_SLIDE) * SLIDE_WEIGHT);
    }

    /**
     * Updates PID loop/motor power.
     * Ensure this is called when using lift, otherwise nothing will happen.
     */
    public void update(){
        recalculateWeight();
        double power = pid.calculate(getPosition(), getForwardFeedValue());
        liftLeft.setPower(power);
        liftRight.setPower(power);
    }

    /**
     * Increases/decreases a PID tuning value by a set amount
     * @param Kp [double] Increment to increase Kp by
     * @param Ki [double] Increment to increase Ki by
     * @param Kd [double] Increment to increase Kd by
     */
    public void adjustPID(double Kp, double Ki, double Kd){
        double[] k = pid.getPIDValues();
        pid.setKp(k[0] + Kp);
        pid.setKi(k[1] + Ki);
        pid.setKd(k[2] + Kd);
    }

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

    public PIDController getPid(){
        return pid;
    }
//    /**
//     * Resets the motor encoder of the passed motor.
//     * @param liftMotor [DcMotor] Motor encoder that should be reset.
//     */
//    private void resetLift(DcMotor liftMotor){
//        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        targetPosition = liftMotor.getCurrentPosition();
//        liftMotor.setTargetPosition(targetPosition);
//        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        liftMotor.setPower(POWER);
//    }
}
