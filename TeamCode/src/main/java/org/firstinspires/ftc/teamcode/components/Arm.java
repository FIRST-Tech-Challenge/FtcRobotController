package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Arm {
    public static DcMotor leftLift;
    public static DcMotor rightLift;
    public static CRServo gripper;

    public static final int lowJunction = 200;
    public static final int middleJunction = 430;
    public static final int highJunction = 610;
    public static int armTarget = 0;

    public Arm(){}

    public static void init(DcMotor lLift, DcMotor rLift, CRServo g){
        Arm.leftLift = lLift;
        Arm.rightLift = rLift;
        Arm.gripper = g;

        leftLift.setDirection(DcMotor.Direction.REVERSE);

        //resetting encoders at home level
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //setting the motors into the necessary mode for using the encoders
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public static void openGripper() {
        gripper.setPower(1);
    }

    public static void closeGripper() {
        gripper.setPower(-1);
    }

    public static void raiseArm(double power) {
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftLift.setPower(power);
        rightLift.setPower(power);

        armTarget = getCurrentPosition();
    }

    public static void dropArm() {
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftLift.setPower(0);
        rightLift.setPower(0);

        armTarget = getCurrentPosition();
    }

    public static void runToPosition(int position) {
        armTarget = position;

        leftLift.setTargetPosition(armTarget);
        rightLift.setTargetPosition(armTarget);

        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (rightLift.isBusy() && leftLift.isBusy()){
            if (getCurrentPosition() < armTarget) {
                if (getCurrentPosition() < middleJunction) {
                    leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                } else {
                    leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
                leftLift.setPower(0.0);
                rightLift.setPower(0.0);
            } else {
                leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftLift.setPower(1.0);
                rightLift.setPower(1.0);
            }
        }
    }

    public static int getCurrentPosition(){ return (leftLift.getCurrentPosition() + rightLift.getTargetPosition()) / 2; }

    public static double getPower(){
        return (leftLift.getPower() + rightLift.getPower()) / 2;
    }
}
