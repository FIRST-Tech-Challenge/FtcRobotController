package org.firstinspires.ftc.teamcode.src.robotAttachments.DriveTrains;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BasicDrivetrain {
    public DcMotor front_right;
    public DcMotor front_left;
    public DcMotor back_right;
    public DcMotor back_left;

    public BasicDrivetrain(HardwareMap hardwareMap, String frontRight, String frontLeft, String backRight, String backLeft){
        front_right = hardwareMap.dcMotor.get(frontRight);
        front_left = hardwareMap.dcMotor.get(frontLeft);
        back_right = hardwareMap.dcMotor.get(backRight);
        back_left = hardwareMap.dcMotor.get(backLeft);

        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        back_right.setDirection(DcMotorSimple.Direction.REVERSE);
        front_right.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    protected BasicDrivetrain(){}

    public void turnRight(double power){
        front_right.setPower(-power);
        back_right.setPower(-power);

        front_left.setPower(power);
        back_left.setPower(power);
    }

    public void turnLeft(double power){
        front_right.setPower(power);
        back_right.setPower(power);

        front_left.setPower(-power);
        back_left.setPower(-power);
    }

    public BasicDrivetrain(DcMotor front_right,DcMotor front_left,DcMotor back_right, DcMotor back_left){
        this.back_left = back_left;
        this.back_right = back_right;
        this.front_right = front_right;
        this.front_left = front_left;
    }

    public void reinitializeMotors(){
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void stopAll() {
        back_right.setPower(0);
        back_left.setPower(0);
        front_right.setPower(0);
        front_left.setPower(0);
    }

    public void strafeAtAngle(double angle, double power) {
        power = boundNumber(power);
        double power1;
        double power2;

        angle = angle % 360;

        power1 = -Math.cos(Math.toRadians(angle + 45.0));
        power2 = -Math.cos(Math.toRadians(angle - 45));

        power1 = power * power1;
        power2 = power * power2;

        front_right.setPower(power1);
        back_left.setPower(power1);

        front_left.setPower(power2);
        back_right.setPower(power2);

    }

    private static double boundNumber(double num) {
        if (num > 1) {
            num = 1;
        }
        if (num < -1) {
            num = -1;
        }
        return num;
    }
}
