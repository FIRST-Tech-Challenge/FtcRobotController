package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;

public class MecanumDrive {
    private final DcMotor rightFront, rightRear, leftFront, leftRear;
    private double maxPower;
    private double rightFrontPower, rightRearPower, leftFrontPower, leftRearPower, powerBase;

    public MecanumDrive(double maxPower, DcMotor rightFront, DcMotor rightRear, DcMotor leftFront, DcMotor leftRear) {
        this.maxPower = maxPower;
        this.rightFront = rightFront;
        this.rightRear = rightRear;
        this.leftFront = leftFront;
        this.leftRear = leftRear;
        this.rightFront.setDirection(DcMotor.Direction.REVERSE);
        this.rightRear.setDirection(DcMotor.Direction.REVERSE);
        this.leftFront.setDirection(DcMotor.Direction.FORWARD);
        this.leftRear.setDirection(DcMotor.Direction.FORWARD);
        this.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public double getMaxPower() {
        return this.maxPower;
    }

    public void setMaxPower(double power) {
        this.maxPower = power;
    }

    public void addPowers(double rightFront, double rightRear, double leftFront, double leftRear) {
        if (rightFront == 0 && rightRear == 0 && leftFront == 0 && leftRear == 0) {
            return;
        }
        this.rightFrontPower += rightFront;
        this.rightRearPower += rightRear;
        this.leftFrontPower += leftFront;
        this.leftRearPower += leftRear;
        this.powerBase += 1;
    }

    public void updatePowers() {
        if (this.powerBase <= 0) {
            this.rightFront.setPower(0);
            this.rightRear.setPower(0);
            this.leftFront.setPower(0);
            this.leftRear.setPower(0);
            return;
        }
        this.rightFront.setPower(this.rightFrontPower / this.powerBase * this.maxPower);
        this.rightRear.setPower(this.rightRearPower / this.powerBase * this.maxPower);
        this.leftFront.setPower(this.leftFrontPower / this.powerBase * this.maxPower);
        this.leftRear.setPower(this.leftRearPower / this.powerBase * this.maxPower);
        this.rightFrontPower = 0;
        this.rightRearPower = 0;
        this.leftFrontPower = 0;
        this.leftRearPower = 0;
        this.powerBase = 0;
    }

    /**
     * @param x left-right shift
     * @param y forward-back shift
     */
    public void shift(double x, double y) {
        double power = Math.sqrt(x * x + y * y);
        if (power < 0.001) {
            return;
        }
        double yaw = Math.atan2(y, x) - Math.PI / 4;
        double xPower = Math.sin(yaw) * power;
        double yPower = Math.cos(yaw) * power;
        this.addPowers(xPower, yPower, xPower, yPower);
    }

    /**
     * @param speed the speed to rotate clockwise
     */
    public void rotate(double speed) {
        if (Math.abs(speed) < 0.01) {
            return;
        }
        this.addPowers(speed, speed, -speed, -speed);
    }
}
