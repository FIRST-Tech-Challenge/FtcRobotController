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
    private double ARM_SPEED = 0.001; //Rotation

    public double MAX_POWER = 0.5;

    private double TICKS_PER_ROTATION = 2048;

    private double MINIMUM_ROTATION = 0; //0 degrees (Relative to starting position)
    private double MAXIMUM_ROTATION = (100 / 360) * TICKS_PER_ROTATION; //100 degrees (Relative to starting position)

    //Internal variables
    private CRServo armLeft, armRight, pivot;

    private Encoder encoder;
    private PIDController pid;
    private int targetPosition;

    public Arm(HardwareMap hw){

        this(hw, "armLeft", "armRight", "FRM");
    }
    public Arm(HardwareMap hw, String nameLeft, String nameRight, String nameEncoder){
        armLeft = hw.get(CRServo.class, nameLeft);
        armRight = hw.get(CRServo.class, nameRight);
        encoder = new OverflowEncoder(new RawEncoder(hw.get(DcMotorEx.class, nameEncoder)));
        armRight.setDirection(DcMotorSimple.Direction.REVERSE);
        pid = new PIDController(0,0,0,0);
        pid.setTarget(getTicks());
    }

    public void changeHeight(double power){
        double target = pid.getTarget() + power * ARM_SPEED;
        target = clamp(target, MAXIMUM_ROTATION, MINIMUM_ROTATION);
        pid.setTarget(target);
    }
    private double clamp(double value, double max, double min){
        return Math.max( min , Math.min( max , value));
    }

    public double getTicks(){
        return encoder.getPositionAndVelocity().position;
    }

    public double getForwardFeedValue(){
        return 0;
    }

    /**
     * This ensures that the PID loops properly.
     * Please add this into the TeleOp loop when using this.
     * For autonomous implementation... Refer to how Roadrunner uses PID
     */
    public void update(){
        double power = pid.calculate(getTicks(), getForwardFeedValue());
        armLeft.setPower(power);
        armRight.setPower(power);
    }

    /**
     * @return [double] The rotational position in DEGREES
     */
    public double getRotation(){
        return getTicks() / TICKS_PER_ROTATION * 360;
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
    public void adjustPID(double Kp, double Ki, double Kd){
        double[] k = pid.getPIDValues();
        pid.setKp(k[0] + Kp);
        pid.setKi(k[1] + Ki);
        pid.setKd(k[2] + Kd);
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
                getRotation(),
                getTicks(),
                targetPosition,
                pid.toString());
    }

}
