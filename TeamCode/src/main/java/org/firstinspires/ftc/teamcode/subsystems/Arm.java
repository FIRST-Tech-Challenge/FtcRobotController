package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.PIDController;

public class Arm {

    //Adjustable Constants
    private double ARM_SPEED = 0.001; //Rotation

    public double MAX_POWER = 0.5;

    private double TICKS_PER_ROTATION = 2048;

    private double MINIMUM_ROTATION = 0; //degrees
    private double MAXIMUM_ROTATION = 100; //degrees

    //Internal variables
    private CRServo armLeft, armRight;

    private Encoder encoder;
    private PIDController pid;
    private int targetPosition;

    public Arm(HardwareMap hw){
        //TODO name encoder according to wiring described by Ryan
        this(hw, "armLeft", "armRight", "FRM");
    }
    public Arm(HardwareMap hw, String nameLeft, String nameRight, String nameEncoder){
        armLeft = hw.get(CRServo.class, nameLeft);
        armRight = hw.get(CRServo.class, nameRight);
        encoder = new OverflowEncoder(new RawEncoder(hw.get(DcMotorEx.class, nameEncoder)));
        armRight.setDirection(DcMotorSimple.Direction.REVERSE);
        pid = new PIDController(0,0,0);
        pid.setTarget(getTicks());
    }

    public void changeHeight(double power){
        pid.setTarget(pid.getTarget() + power * ARM_SPEED);
    }

    public double getTicks(){
        return encoder.getPositionAndVelocity().position;
    }

    /**
     * This ensures that the PID loops properly.
     * Please add this into the TeleOp loop when using this.
     * For autonomous implementation... Refer to how Roadrunner uses PID
     */
    public void update(){
        double power = pid.calculate(getTicks());
        armLeft.setPower(power);
        armRight.setPower(power);
    }

    /**
     * @return [double] The rotational position in degrees
     */
    public double getRotation(){
        return getTicks() / 360;
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
    @Override
    public String toString(){
        return String.format("Arm current Rotation: %f\n" +
                "Arm current position: %f\n" +
                "Arm target position: %f\n" +
                "Arm PID Data: \n%s",
                getRotation(),
                getTicks(),
                targetPosition,
                pid.toString());
    }

}
