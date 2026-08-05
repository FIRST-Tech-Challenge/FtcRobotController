package org.firstinspires.ftc.teamcode.PRL.Class;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TurretClass {

    private DcMotorEx turretMotor;

    private static final double MOTOR_TICKS_PER_REV = 383.6;
    private static final double GEAR_RATIO = 150.0 / 24.0;
    private static final double TICKS_PER_REV = MOTOR_TICKS_PER_REV * GEAR_RATIO;

    private double targetAngle = 0;
    private double continuousAngle = 0;
    private double lastRawAngle = 0;

    // 케이블 보호 범위
    private static final double MAX_ANGLE = 540;
    private static final double MIN_ANGLE = -540;

    // PID
    private double kP = 0.012;
    private double kI = 0;
    private double kD = 0.001;

    private double integral = 0;
    private double lastError = 0;

    public TurretClass(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");

        turretMotor.setDirection(DcMotor.Direction.REVERSE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void update() {
        updateAngle();
        cableManagement();

        double error = targetAngle - continuousAngle;

        // 최단 방향
        while (error > 180)
            error -= 360;
        while (error < -180)
            error += 360;

        integral += error;

        double derivative = error - lastError;
        lastError = error;

        double power = kP * error + kI * integral + kD * derivative;
        power = Math.max(-1, Math.min(1, power));

        turretMotor.setPower(power);
    }

    private void updateAngle() {
        double rawAngle = (turretMotor.getCurrentPosition() / TICKS_PER_REV) * 360.0;
        double delta = rawAngle - lastRawAngle;

        if (delta > 180)
            delta -= 360;
        if (delta < -180)
            delta += 360;

        continuousAngle += delta;
        lastRawAngle = rawAngle;
    }

    private void cableManagement() {
        if (continuousAngle > MAX_ANGLE) {
            continuousAngle -= 360;
            targetAngle -= 360;
        } else if (continuousAngle < MIN_ANGLE) {
            continuousAngle += 360;
            targetAngle += 360;
        }
    }

    public void setTargetAngle(double angle) {
        targetAngle = angle;
    }

    public double getAngle() {
        return continuousAngle;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public void setPower(double power) {
        turretMotor.setPower(power);
    }

    public void stop() {
        turretMotor.setPower(0);
    }
}
