package org.firstinspires.ftc.teamcode.purepursuit.localization;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public final class DeadWheel {
    private final DcMotorImplEx motor;
    private int direction = 1;

    public DeadWheel(@NonNull DcMotorImplEx motor) {
        this.motor = motor;

        reset();
    }

    /**
     * Reverses the direction of the dead wheel. This reversal is independent from the underlying
     * motor
     */
    public void reverse() {
        direction = -direction;
    }

    /**
     * @return The direction of the dead wheel, 1 = forward, -1 = reverse
     */
    public int direction() {
        return direction;
    }

    /**
     * @return The current position of the dead wheel multiplied by the direction of the motor, see
     *         {@link DeadWheel#direction()}
     */
    public int ticks() {
        return motor.getCurrentPosition() * direction;
    }

    /**
     * @return The velocity of the motor in ticks per second.
     */
    public double velocity() {
        return motor.getVelocity();
    }

    public void reset() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void debug(@NonNull Telemetry telemetry) {
        telemetry.addData("Direction", direction);
        telemetry.addData("Ticks", motor.getCurrentPosition() * direction);
        telemetry.addData("Velocity (Ticks Per Second)", motor.getVelocity());
    }
}
