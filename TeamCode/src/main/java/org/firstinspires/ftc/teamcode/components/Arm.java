package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Arm {
    public DcMotor leftLift;
    public DcMotor rightLift;
    public Servo gripper;

    //TODO: CHANGE VALUE TO 420, 630, 910 FOR 11166-RC!!!!
    //150, 300, 450 for test robot
    public final int lowJunction = 420;
    public final int middleJunction = 630;
    public final int highJunction = 910;
    public int armTarget = 0;

    public enum ManualArm {drop, raise, none};

    public ManualArm manualArm = ManualArm.none;

    public Arm(DcMotor lLift, DcMotor rLift, Servo g){
        this.leftLift = lLift;
        this.rightLift = rLift;
        this.gripper = g;
    }

    public void init(){
        gripper.setPosition(0.75);

        //TODO: REVERSE rightLift FOR 11166-RC!!!
        rightLift.setDirection(DcMotor.Direction.REVERSE);

        //resetting encoders at home level
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //setting the motors into the necessary mode for using the encoders
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void openGripper() {
        gripper.setPosition(0.73);
    }

    public void closeGripper() { gripper.setPosition(0.48); }

    public void runToPosition(int position) {
        armTarget = position;

        leftLift.setTargetPosition(armTarget);
        rightLift.setTargetPosition(armTarget);

        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(rightLift.isBusy() && leftLift.isBusy()) {
            if (leftLift.getCurrentPosition() < leftLift.getTargetPosition() && rightLift.getCurrentPosition() < rightLift.getTargetPosition()) {
                leftLift.setPower(1.0);
                rightLift.setPower(1.0);
            } else {
                leftLift.setPower(0.0);
                rightLift.setPower(0.0);
            }
        }
    }

    public void fall() {
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while(getCurrentPosition() > (lowJunction + middleJunction)/2) {
            leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            leftLift.setPower(0.0);
            rightLift.setPower(0.0);
        }

        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setPower(0.0);
        rightLift.setPower(0.0);
    }

    public void setArmPower(Gamepad gamepad, double power) {
        boolean drop = manualArm == ManualArm.drop;
            if (manualArm == ManualArm.none) {
                leftLift.setTargetPosition(armTarget);
                rightLift.setTargetPosition(armTarget);

                leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else {
                leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                leftLift.setPower((drop) ? 0.0 : power * 0.7);
                rightLift.setPower((drop) ? 0.0 : power * 0.7);

                armTarget = getCurrentPosition();
            }

            if (rightLift.isBusy() && leftLift.isBusy()){
                if (getCurrentPosition() > armTarget) {
                    if (getCurrentPosition() > middleJunction && getTargetPosition() != middleJunction) {
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
                    leftLift.setPower(power);
                    rightLift.setPower(power);
                }
            }
    }

    public void armTriggers(Gamepad gamepad) {
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (gamepad.left_trigger > 0) {
            armTarget = getCurrentPosition();

            leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            leftLift.setPower(0.0);
            rightLift.setPower(0.0);
        }

        if (gamepad.right_trigger > 0) {
            armTarget = getCurrentPosition();

            leftLift.setPower(1.0);
            rightLift.setPower(1.0);
        }
    }

    public int getCurrentPosition(){ return (leftLift.getCurrentPosition() + rightLift.getCurrentPosition()) / 2; }

    public int getTargetPosition(){ return (leftLift.getTargetPosition() + rightLift.getTargetPosition()) / 2; }

    public double getPower(){ return (leftLift.getPower() + rightLift.getPower()) / 2; }
}
