package org.firstinspires.ftc.forteaching;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class CRServoCode {
    public class CRServoConstant{
        // no need to have import statement of this class
        public static final double MAX_POSITIVE_SPEED = 1.0;
        public static final double MAX_NEGATIVE_SPEED = -1.0;
        public static final double STOP_SPEED = 0.0;
    }

    private final CRServo servo;

    public CRServoCode(CRServo servo) {
        this.servo = servo;
    }

    public double getPower() {
        return this.servo.getPower();
    }

    public void setPower(double power) {
        this.servo.setPower(power);
    }

    public DcMotorSimple.Direction getDirection() {
        return this.servo.getDirection();
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        // only work in the init() method, not the loop() method
        this.servo.setDirection(direction);
    }

    public void invertDirection() {
        // not useful since you can't change direction the loop() method
        this.setDirection(this.servo.getDirection() == DcMotorSimple.Direction.FORWARD ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    }

    public void stop() {
        this.setPower(CRServoConstant.STOP_SPEED);
    }

    public void setMaxPositiveSpeed() {
        this.setPower(CRServoConstant.MAX_POSITIVE_SPEED);
    }

    public void setMaxNegativeSpeed() {
        this.setPower(CRServoConstant.MAX_NEGATIVE_SPEED);
    }
}
