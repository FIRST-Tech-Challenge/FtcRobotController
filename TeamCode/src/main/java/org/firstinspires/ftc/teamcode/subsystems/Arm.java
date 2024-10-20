package org.firstinspires.ftc.teamcode.subsystems;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.CRServo;
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
    private double ARM_SPEED = 10; //Ticks

    public double MAX_POWER = 0.5;

    private double TICKS_PER_ROTATION = 8192;

    private double MINIMUM_ROTATION = 0; //0 degrees (Relative to starting position)
    private double MAXIMUM_ROTATION = 3000; //Ticks

    //Internal variables
    private CRServo armLeft, armRight;

    private Encoder encoder;
    private PIDController pid;

    private int targetPosition;

    public Arm(HardwareMap hw){

        this(hw, "armLeft", "armRight", "liftLeft");
    }
    public Arm(HardwareMap hw, String nameLeft, String nameRight, String nameEncoder){
        armLeft = hw.get(CRServo.class, nameLeft);
        armRight = hw.get(CRServo.class, nameRight);
        encoder = new OverflowEncoder(new RawEncoder(hw.get(DcMotorEx.class, nameEncoder)));
        encoder.setDirection(DcMotorSimple.Direction.REVERSE);
        armRight.setDirection(DcMotorSimple.Direction.REVERSE);
        armLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        pid = new PIDController(0,0,0,0);
        pid.setTarget(getTicks());
    }

    public void changeHeight(double power){
        targetPosition +=  (power * ARM_SPEED);
//        targetPosition = clamp(targetPosition, MAXIMUM_ROTATION, MINIMUM_ROTATION);
        pid.setTarget(targetPosition);
    }
    private int clamp(double value, double max, double min){
        return (int) Math.max( min , Math.min( max , value));
    }

    public double getTicks(){
        return encoder.getPositionAndVelocity().position;
    }

    public double getForwardFeedValue(){
        return Math.cos(Math.toRadians(getRotation()));
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
        return (getTicks() / TICKS_PER_ROTATION) * 360;
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

    @SuppressLint("DefaultLocale")
    @Override
    public String toString(){
        return String.format("Arm current Rotation: %f\n" +
                "Arm current position: %f\n" +
                "Arm target position: %d\n" +
                "Arm PID Data: \n%s",
                this.getRotation(),
                this.getTicks(),
                this.targetPosition,
                pid.toString());
    }

}
