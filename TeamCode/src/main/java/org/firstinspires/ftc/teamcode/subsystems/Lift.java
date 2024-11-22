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
    private Encoder encoder;

//    public Lift(HardwareMap hw){
//        this(hw, "lift", "lift");
//    }

    /**
     * Primary constructor for the Lift Subsystem Class
     * @param hw [HardwareMap] Hardware map necessary to initialize motors.
     * @param nameLift [String] Name of the lift motor assigned in the configuration.
     * @param nameEncoder [String] Name of the encoder assigned in the configuration.
     */
    public Lift(HardwareMap hw, String leftLiftName, String rightLiftName, String nameEncoder){
        leftLift = hw.get(DcMotor.class, leftLiftName);
        rightLift = hw.get(DcMotor.class, rightLiftName);
        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);
//        encoder = new OverflowEncoder(new RawEncoder(hw.get(DcMotorEx.class, nameEncoder)));
//
//        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftLift.setPower(0);
//
//        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightLift.setPower(0);
//        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        lift.setPower(1);
    }
    public void moveLift(double power){
        leftLift.setPower(power);
        rightLift.setPower(power);
    }
    public double getPosition(){
        return encoder.getPositionAndVelocity().position;
    }
    public void setPosition(int targetPosition){
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLift.setTargetPosition(targetPosition);
        leftLift.setPower(1.0);

        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setTargetPosition(targetPosition);
        rightLift.setPower(1.0);
    }
}
