package org.firstinspires.ftc.teamcode.subsystems;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
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


    //Values: 590, 1200, 2700
    public int ZERO = 0; //Ticks //SAMPLE INTAKE
    public int SPECIMIN_INTAKE = 590; //Ticks //SPECIMIN INTAKE
    public int HIGH_BAR = 2700; //Ticks //SPECIMIN DESPOSIT
    public int MAX_HEIGHT = 4300; //Ticks //SAMPLE DEPOSIT

    public double GRAVITY = 9.8; // N/kgs
    public double ARM_WEIGHT = 1;//kgs
    public double SLIDE_WEIGHT = 0.3; //kgs
    public double TICKS_PER_SLIDE = 500; //ticks - Figure out later
    public double calculatedWeight = GRAVITY * ARM_WEIGHT;

    //Internal variables
    private DcMotorEx liftLeft, liftRight;
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
        this.liftLeft = hw.get(DcMotorEx.class, nameLeft);
        this.liftRight = hw.get(DcMotorEx.class, nameRight);
        encoder = new OverflowEncoder(new RawEncoder(hw.get(DcMotorEx.class, nameEncoder)));
        //Reverse one of the motors
        this.liftRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //Ensures motor encoders are reset

        pid = new PIDController(0.033,0,0.0004,0);
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

    public double getTargetPosition(){
        return targetPosition;
    }
    /**
     * Sets the motors' target position to [LOW_HEIGHT]
     */
    public void goToSampleIntake(){
        this.targetPosition = ZERO;
        pid.setTarget(this.targetPosition);
    }

    public void goToTopBucket(){
        this.targetPosition = MAX_HEIGHT;
        pid.setTarget(this.targetPosition);
    }

    /**
     * Sets the motors' target position to [MEDIUM_HEIGHT]
     */
    public void goToSpeciminIntake(){
        this.targetPosition = SPECIMIN_INTAKE;
        pid.setTarget(this.targetPosition);
    }

    /**
     * Sets the motors' target position to [HIGH_HEIGHT]
     */
    public void goToHighBar(){
        this.targetPosition = HIGH_BAR;
        pid.setTarget(this.targetPosition);
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
    public double update(){
        recalculateWeight();
        double power = pid.calculate(getPosition(), getForwardFeedValue());
        liftLeft.setPower(power);
        liftRight.setPower(power);
        return power;
    }

    /**
     * Increases/decreases a PID tuning value by a set amount
     * @param Kp [double] Increment to increase Kp by
     * @param Ki [double] Increment to increase Ki by
     * @param Kd [double] Increment to increase Kd by
     */
    public void adjustPID(double Kp, double Ki, double Kd){
//        double[] k = pid.getPIDValues();
        pid.setKp(Kp);
        pid.setKi(Ki);
        pid.setKd(Kd);
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

    @SuppressLint("DefaultLocale")
    public String getArmCurrent(){
        return String.format(
                "Left Arm Current: %f\n" +
                        "Right Arm Current: %f",
                liftLeft.getCurrent(CurrentUnit.AMPS),
                liftRight.getCurrent(CurrentUnit.AMPS)
        );
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
