package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift{
    public int Lift_SPEED = 20;

    private DcMotor leftLift;
    private DcMotor rightLift;
    private Encoder leftEncoder;
    private Encoder rightEncoder;

//    public Lift(HardwareMap hw){
//        this(hw, "lift", "lift");
//    }

    /**
     * Primary constructor for the Lift Subsystem Class
     * @param hw [HardwareMap] Hardware map necessary to initialize motors.
     * @param leftLiftName [String] Name of the left lift motor assigned in the configuration.
     * @param rightLiftName [String] Name of the right lift motor assigned in the configuration.
     */
    public Lift(HardwareMap hw, String leftLiftName, String rightLiftName, String leftEncoderName, String rightEncoderName){
        leftLift = hw.get(DcMotor.class, leftLiftName);
        rightLift = hw.get(DcMotor.class, rightLiftName);
        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);
        leftEncoder = new OverflowEncoder(new RawEncoder(hw.get(DcMotorEx.class, leftEncoderName)));
        rightEncoder = new OverflowEncoder(new RawEncoder(hw.get(DcMotorEx.class, rightEncoderName)));

        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftLift.setTargetPosition(0);
        rightLift.setTargetPosition(0);

        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightLift.setPower(1);
        leftLift.setPower(1);

    }
    public Lift(HardwareMap hw, String leftLiftName, String rightLiftName){
        leftLift = hw.get(DcMotor.class, leftLiftName);
        rightLift = hw.get(DcMotor.class, rightLiftName);
        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void moveLift(double power){
        leftLift.setPower(power);
        rightLift.setPower(power);
    }
    public double getPosition(){
        return leftEncoder.getPositionAndVelocity().position;
    }
    public void setPosition(int targetPosition){

        leftLift.setTargetPosition(targetPosition);
        rightLift.setTargetPosition(targetPosition);

        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftLift.setPower(1.0);
        rightLift.setPower(1.0);
    }
}
