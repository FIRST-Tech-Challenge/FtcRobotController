package Team.BasicRobots;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.*;

public class BasicMecanum {

    public DcMotor RFMotor;
    public DcMotor RBMotor;
    public DcMotor LFMotor;
    public DcMotor LBMotor;

    public double motorOffset = 0;

    public double mult = 0.29;

    public void init(HardwareMap Map) {

        LFMotor = Map.dcMotor.get("LFMotor");
        LBMotor = Map.dcMotor.get("LBMotor");
        RFMotor = Map.dcMotor.get("RFMotor");
        RBMotor = Map.dcMotor.get("RBMotor");

        LFMotor.setPower(0.0);
        LBMotor.setPower(0.0);
        RFMotor.setPower(0.0);
        RBMotor.setPower(0.0);

        //TODO: Figure out which motors need to be reversed, etc. so that the robot actually goes forward lmao

        LFMotor.setDirection(DcMotor.Direction.FORWARD);
        RFMotor.setDirection(DcMotor.Direction.FORWARD);
        LBMotor.setDirection(DcMotor.Direction.FORWARD);
        RBMotor.setDirection(DcMotor.Direction.FORWARD);

        //for now, we do this (maybe change later-
//        LFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        RFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        LBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        RBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//
        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        LFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //:crab: william is gone :crab:
    }

    public void moveStraight(double power) {
        LFMotor.setPower(power * mult);
        RFMotor.setPower(power);
        LBMotor.setPower(power * mult);
        RBMotor.setPower(power);
    }

    public void moveLeft(double power) {
        LFMotor.setPower(-power * mult);
        RFMotor.setPower(power);
        LBMotor.setPower(power * mult);
        RBMotor.setPower(-power);
    }

    public void moveRight(double power) {
        LFMotor.setPower(power * mult);
        RFMotor.setPower(-power);
        LBMotor.setPower(-power * mult);
        RBMotor.setPower(power);
    }

    public void moveBackwards(double power) {
        LFMotor.setPower(-power * mult);
        RFMotor.setPower(-power);
        LBMotor.setPower(-power * mult);
        RBMotor.setPower(-power);
    }

    public void stop() {
        LFMotor.setPower(0);
        RFMotor.setPower(0);
        LBMotor.setPower(0);
        RBMotor.setPower(0);
    }

    public void pivotTurn(double power, boolean rightBumper, boolean leftBumper) {
        power = power*0.5;
        if(rightBumper && leftBumper) {
            RFMotor.setPower(0);
            LFMotor.setPower(0);
            RBMotor.setPower(0);
            LBMotor.setPower(0);
        } else if(leftBumper) {
            RFMotor.setPower(power);
            LFMotor.setPower(-power * mult);
            RBMotor.setPower(power);
            LBMotor.setPower(-power * mult);
        } else if(rightBumper) {
            RFMotor.setPower(-power);
            LFMotor.setPower(power * mult);
            RBMotor.setPower(-power);
            LBMotor.setPower(power * mult);
        }
    }

    public void octoStrafe(double power, boolean up, boolean down, boolean left, boolean right) {
        if (up) {
            if (right) {
                RFMotor.setPower(power);
                LFMotor.setPower(0);
                RBMotor.setPower(0);
                LBMotor.setPower(power * mult);
            } else if (left) {
                RFMotor.setPower(0);
                LFMotor.setPower(power * mult);
                RBMotor.setPower(power);
                LBMotor.setPower(0);
            } else {
                RFMotor.setPower(power);
                LFMotor.setPower(power * mult);
                RBMotor.setPower(power);
                LBMotor.setPower(power * mult);
            }
        } else if (down) {
            if (right) {
                RFMotor.setPower(0);
                LFMotor.setPower(-power * mult);
                RBMotor.setPower(-power);
                LBMotor.setPower(0);
            } else if (left) {
                RFMotor.setPower(-power);
                LFMotor.setPower(0);
                RBMotor.setPower(0);
                LBMotor.setPower(-power * mult);
            } else {
                RFMotor.setPower(-1);
                LFMotor.setPower(-power * mult);
                RBMotor.setPower(-1);
                LBMotor.setPower(-power * mult);
            }
        }
        else {
            if (right) {
                RFMotor.setPower(-power);
                LFMotor.setPower(power * mult);
                RBMotor.setPower(power);
                LBMotor.setPower(-power * mult);
            } else if (left) {
                RFMotor.setPower(power);
                LFMotor.setPower(-power * mult);
                RBMotor.setPower(-power);
                LBMotor.setPower(power * mult);
            }
        }
    }
}