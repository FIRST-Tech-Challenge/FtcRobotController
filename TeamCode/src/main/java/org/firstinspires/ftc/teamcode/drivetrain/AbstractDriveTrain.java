package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.AppContext;
import org.firstinspires.ftc.teamcode.measure.Imu;
import org.firstinspires.ftc.teamcode.util.Constants;

public abstract class AbstractDriveTrain implements DriveTrain {
    private Imu imu;

    public void setImu(Imu imu) {
        this.imu = imu;
    }

    public Imu getImu() {
        return imu;
    }

    public void setDirection(DcMotor motor, DcMotor.Direction direction) {
        if (motor != null) {
            motor.setDirection(direction);
        }
    }

    public void setZeroPowerBehavior(DcMotor motor, DcMotor.ZeroPowerBehavior powerBehavior) {
        if (motor != null) {
            motor.setZeroPowerBehavior(powerBehavior);
        }
    }

    public void setMode(DcMotor motor, DcMotor.RunMode runMode) {
        if (motor != null) {
            motor.setMode(runMode);
        }
    }

    public void setTargetPosition(DcMotor motor, int targetPosition) {
        motor.setTargetPosition(targetPosition);
    }

    @Override
    public void stopMotor(DcMotor motor) {
        setPower(motor, Constants.ZERO_POWER);
    }

    @Override
    public void setPower(DcMotor motor, double power) {
        motor.setPower(power);
    }

    public boolean isOpModeActive() {
        return AppContext.getInstance().isOpModeActive();
    }

    public boolean isStopRequested() {
        return AppContext.getInstance().isStopRequested();
    }
}
