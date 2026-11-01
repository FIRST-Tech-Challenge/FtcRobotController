package org.firstinspires.ftc.teamcode.follower.hardware;

import org.firstinspires.ftc.teamcode.follower.utils.Utils;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class CachedMotor {
    private final DcMotorEx motor;

    private double power = 0;
    private final double powerThreshold;
    private DcMotor.ZeroPowerBehavior zeroPowerBehavior;
    private DcMotorSimple.Direction direction;

    public CachedMotor(DcMotorEx motor, double powerThreshold) {
        this.motor = motor;
        this.powerThreshold = powerThreshold;
    }

    public void setPower(double power) {
        if (Double.isNaN(power) || Double.isInfinite(power)) return;

        double desired = Utils.clamp(power, -1, 1);

        boolean exceedsThreshold = Math.abs(this.power - desired) >= powerThreshold;
        boolean switchesSigns = Math.signum(this.power) != Math.signum(desired);

        if (exceedsThreshold || switchesSigns) {
            this.power = desired;
            motor.setPower(desired);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        if (zeroPowerBehavior != behavior) {
            zeroPowerBehavior = behavior;
            motor.setZeroPowerBehavior(behavior);
        }
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        if (this.direction != direction) {
            this.direction = direction;
            motor.setDirection(direction);
        }
    }

    public DcMotorEx raw() {
        return motor;
    }
}