package org.firstinspires.ftc.teamcode.subsystems;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.PIDController;

public class Arm {

    /*TODO LIST:
        Identify [Rotational/Tick Limits] of Arm
        Tune PID Parameters accordingly
    */


    //Adjustable Constants
    private double ARM_SPEED = 50; //Ticks

    public double MAX_POWER = 0.5;

    private double TICKS_PER_ROTATION = 8192;

    private double MINIMUM_ROTATION = 0; //0 degrees (Relative to starting position)
    private double MAXIMUM_ROTATION = 3000; //Ticks

    //Internal variables
    private CRServo armLeft, armRight;

    //Position values 50, wrist, 3200

    private int BASE_HEIGHT = 3400;//3450 - 50; //ticks
    private int SPECIMIN_HEIGHT = 2420;//3450 - 800; //ticks

    private int REST_HEIGHT = 200;//3450 - 3250; //ticks
    private int DEPOSIT_HEIGHT = -220;//3450 - 3670; //ticks 3670

    enum ArmState{
        BASE_HEIGHT,
        SPECIMIN_HEIGHT,
        REST_HEIGHT,
        DEPOSIT_HEIGHT
    }

    private ArmState currentState;


    private Encoder encoder;
    private PIDController pid;

    private double armAngleOffset = -4;

    private int targetPosition;

    public Arm(HardwareMap hw){

        this(hw, "armLeft", "armRight", "liftLeft");
    }
    public Arm(HardwareMap hw, String nameLeft, String nameRight, String nameEncoder){
        armLeft = hw.get(CRServo.class, nameLeft);
        armRight = hw.get(CRServo.class, nameRight);
        DcMotorEx motorPort = hw.get(DcMotorEx.class, nameEncoder);
        //Resets encoder on start
        motorPort.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorPort.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Creates encoder object to use
        encoder = new OverflowEncoder(new RawEncoder(motorPort));



//        encoder.setDirection(DcMotorSimple.Direction.REVERSE);
        armRight.setDirection(DcMotorSimple.Direction.REVERSE);
//        armLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        pid = new PIDController(0.001,0,0.0001,0.25);
        pid.setTarget(getTicks());
        currentState = ArmState.REST_HEIGHT;
    }

    public void changeHeight(double power){
        targetPosition +=  (power * ARM_SPEED);
//        targetPosition = clamp(targetPosition, MAXIMUM_ROTATION, MINIMUM_ROTATION);
        pid.setTarget(targetPosition);
    }
    private int clamp(double value, double max, double min){
        return (int) Math.max( min , Math.min( max , value));
    }

    public int getTicks(){
        return encoder.getPositionAndVelocity().position;
    }

    public double getForwardFeedValue(){
//        return 1;
        return Math.cos(Math.toRadians(getRotation()));
    }

    public void goToBase(){
        currentState = ArmState.BASE_HEIGHT;
        targetPosition = BASE_HEIGHT;
        pid.setTarget(targetPosition);
    }

    public void goToSpecimin(){
        currentState = ArmState.SPECIMIN_HEIGHT;
        targetPosition = SPECIMIN_HEIGHT;
        pid.setTarget(targetPosition);
    }

    public void goToRest(){
        currentState = ArmState.REST_HEIGHT;
        targetPosition = REST_HEIGHT;
        pid.setTarget(targetPosition);
    }

    public void goToDeposit(){
        currentState = ArmState.DEPOSIT_HEIGHT;
        targetPosition = DEPOSIT_HEIGHT;
        pid.setTarget(targetPosition);
    }

    public int getTargetPosition(){
        return targetPosition;
    }


    /**
     * This ensures that the PID loops properly.
     * Please add this into the TeleOp loop when using this.
     * For autonomous implementation... Refer to how Roadrunner uses PID
     */
    public double update(){
        double power = pid.calculate(getTicks(), getForwardFeedValue());
        armLeft.setPower(power);
        armRight.setPower(power);
        return power;
    }

    /**
     * @return [double] The rotational position in DEGREES
     */
    public double getRotation(){
        return (getTicks() / TICKS_PER_ROTATION) * 360  + armAngleOffset;
    }

    public PIDController getPIDObject(){
        return pid;
    }

    /**
     * Increases/decreases a PID tuning value by a set amount
     * @param Kp [double] Increment to increase Kp by
     * @param Ki [double] Increment to increase Ki by
     * @param Kd [double] Increment to increase Kd by
     */
    public void adjustPID(double Kp, double Ki, double Kd, double Kf){
        pid.setKp(Kp);
        pid.setKi(Ki);
        pid.setKd(Kd);
        pid.setKf(Kf);
    }
    public PIDController getPid(){
        return pid;
    }

    public ArmState getState(){
        return currentState;
    }

    @SuppressLint("DefaultLocale")
    @Override
    public String toString(){
        return String.format("Arm current Rotation: %f\n" +
                "Arm current position: %d\n" +
                "Arm target position: %d\n" +
                "Arm PID Data: \n%s",
                this.getRotation(),
                this.getTicks(),
                this.targetPosition,
                pid.toString());
    }

}
