package org.firstinspires.ftc.teamcode.Arm;

import static org.firstinspires.ftc.teamcode.Arm.TubeDriver.ROTATION_OFFSET_TICKS;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Utils.AdvancedPidController;

@Config
public class SlideVerticalArm {
    public static final int ROTATION_INIT_TICKS = 0;

    private static DcMotorEx rotationMotor1, rotationMotor2;

    public double rotationTargetAngle;

    private AdvancedPidController pidController = new AdvancedPidController(
            TubeDriver.ROTATION_P,
            TubeDriver.ROTATION_I,
            TubeDriver.ROTATION_D,
            TubeDriver.ROTATION_I_MAX,
            "Rotation Pid"
    );

    public SlideVerticalArm(DcMotorEx rotationMotor1, DcMotorEx rotationMotor2) {
        SlideVerticalArm.rotationMotor1 = rotationMotor1;
        SlideVerticalArm.rotationMotor2 = rotationMotor2;

        pidController.setOutputLimits(TubeDriver.ROTATION_PID_POWER_LIMIT);

        rotationTargetAngle = ROTATION_INIT_TICKS;
    }

    public void updateMotorPower() {
        double power = getPidMotorPower();

        rotationMotor1.setPower(power);
        rotationMotor2.setPower(power);
    }

    public double getPidMotorPower() {
        double currentAngle = getCurrentAngleDegrees();

        double power = Math.sin(Math.toRadians(currentAngle)) * TubeDriver.ROTATION_Kx;

        double error = currentAngle - rotationTargetAngle;
        if (Math.abs(error) > TubeDriver.ROTATION_T) {
            double pidPower = pidController.calculate(error);
            power += pidPower;

            if (pidPower > 0) {
                power += TubeDriver.ROTATION_Kv;
            } else {
                power -= TubeDriver.ROTATION_Kv;
            }
        }

        return power;
    }

    @Deprecated
    public double getRawPidPower() {
        double error = (getCurrentAngleDegrees() - rotationTargetAngle);
        return pidController.calculate(error);
    }

    public void resetEncoders() {
        rotationMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotationMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rotationMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rotationMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void manualControl(double p) {
        double power = Math.sin(Math.toRadians(getCurrentAngleDegrees())) * TubeDriver.ROTATION_Kx;

        power += p;
        if (p > 0) {
            power += TubeDriver.ROTATION_Kv;
        } else if (p < 0) {
            power -= TubeDriver.ROTATION_Kv;
        }

        rotationMotor1.setPower(power);
        rotationMotor2.setPower(power);
    }


    public void setToAngleDegrees(double theta) {
        rotationTargetAngle = theta;
    }

    public double getCurrentAngleDegrees() {
        return 360 * getCurrentPositionTicks() / TubeDriver.TICKS_PER_TUBE_ROTATION_2;
    }

    public int getCurrentPositionTicks() {
        return (rotationMotor1.getCurrentPosition() - ROTATION_OFFSET_TICKS);
    }

    public void stopMotors() {
        rotationMotor1.setPower(0);
        rotationMotor2.setPower(0);
    }

    public void resetPidValues() {
        pidController = new AdvancedPidController(
                TubeDriver.ROTATION_P,
                TubeDriver.ROTATION_I,
                TubeDriver.ROTATION_D,
                TubeDriver.ROTATION_I_MAX,
                "Rotation Pid"
        );
        pidController.setOutputLimits(TubeDriver.ROTATION_PID_POWER_LIMIT);
    }
}
