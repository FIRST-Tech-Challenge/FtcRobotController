package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MecanumDrive {

    private DcMotor motorBL;
    private DcMotor motorFL;
    private DcMotor motorBR;
    private DcMotor motorFR;

    private double blPower;
    private double flPower;
    private double brPower;
    private double frPower;

    public MecanumDrive(
            DcMotor BL,
            DcMotor FL,
            DcMotor BR,
            DcMotor FR
    ) {
        motorBL = BL;
        motorFL = FL;
        motorBR = BR;
        motorFR = FR;

        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);


        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    };

    public void xInput(double input) {

    }
    public void yInput(double input) {
        blPower += -input;
        flPower += -input;
        brPower += -input;
        frPower += -input;
    }

    public void move(double x, double y, double theta) {

        blPower = -x -y + theta;
        flPower = x -y + theta;
        brPower = x -y - theta;
        frPower = -x -y - theta;

        double coefficient = Math.max(Math.max(blPower, flPower), Math.max(brPower, frPower));
        coefficient = Math.max(1.0, Math.abs(coefficient));

        motorBL.setPower(blPower / coefficient);
        motorFL.setPower(flPower / coefficient);
        motorBR.setPower(brPower / coefficient);
        motorFR.setPower(frPower / coefficient);

    }


    public double blPower() {
        return blPower;
    }
    public double flPower() {
        return flPower;
    }
    public double brPower() {
        return brPower;
    }
    public double frPower() {
        return frPower;
    }

    public enum Side {
        LEFT, RIGHT;
    }
    private DcMotor[] enumToMotor(Side side) {
        switch (side) {
            case LEFT:
                return new DcMotor[]{this.motorFL, this.motorBL};
            case RIGHT:
                return new DcMotor[]{this.motorFR, this.motorBR};
        }; return new DcMotor[]{};
    }
}
