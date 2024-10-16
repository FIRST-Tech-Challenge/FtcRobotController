package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ViperSlide {
    private DcMotor left;
    private DcMotor right;
    private DcMotor pivot;


    public ViperSlide(HardwareMap map) {
        left = map.get(DcMotor.class, "viperLeft");
        right = map.get(DcMotor.class, "viperRight");
        pivot = map.get(DcMotor.class, "pivot");

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left.setDirection(DcMotor.Direction.FORWARD);
        right.setDirection(DcMotor.Direction.REVERSE);
        pivot.setDirection(DcMotor.Direction.REVERSE);
    }

    public void setMotorPowers(double power) {
        left.setPower(power);
        right.setPower(power);
    }

    public void extend(double power) {
        setMotorPowers(power);
    }

    public void retract(double power) {
        setMotorPowers(-power);
    }

    public void stop() {
        setMotorPowers(0);
    }
    public void raise(double power) {
        pivot.setPower(power);
    }

    public void lower(double power) {
        pivot.setPower(-power);
    }

    public void stopPivot() {
        pivot.setPower(0);
    }

}
