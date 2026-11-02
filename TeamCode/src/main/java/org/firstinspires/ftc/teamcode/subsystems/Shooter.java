package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.jetbrains.annotations.NotNull;

/** Owns the shooter motor; the TeleOp decides when to fire. */
@Configurable
public final class Shooter extends SubsystemBase {
    // Shared velocity in encoder ticks per second; tune in Panels.
    public static double SHOOT_VEL = 1560;
    // Copy measured values from Shooter PIDF Tuner, then opt in. Zeros are NOT tuned values.
    public static boolean USE_TUNED_PIDF = false;
    public static double VELOCITY_P = 0;
    public static double VELOCITY_I = 0;
    public static double VELOCITY_D = 0;
    public static double VELOCITY_F = 0;
    private final DcMotorEx shooter;
    private double commandedVel;

    public Shooter(@NotNull final HardwareMap hMap) {
        shooter = hMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setVelocity(0.0);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (USE_TUNED_PIDF) {
            validateCoefficient(VELOCITY_P);
            validateCoefficient(VELOCITY_I);
            validateCoefficient(VELOCITY_D);
            validateCoefficient(VELOCITY_F);
            shooter.setVelocityPIDFCoefficients(VELOCITY_P, VELOCITY_I, VELOCITY_D, VELOCITY_F);
        }
        stop();
    }

    public void shoot() {
        setVelocity(SHOOT_VEL);
    }

    public void setVelocity(double velocity) {
        shooter.setVelocity(velocity);
        commandedVel = velocity;
    }

    public void stop() {
        // Keep velocity commands consistent: SDK power caching can suppress a repeated setPower(0).
        shooter.setVelocity(0.0);
        commandedVel = 0.0;
    }

    /** Last successfully written command, not an RPM or motion measurement. */
    public double getCommandedVel() {
        return commandedVel;
    }

    private static void validateCoefficient(double value) {
        if (!Double.isFinite(value) || value < 0 || value > 32767) {
            throw new IllegalArgumentException("Shooter PIDF must be finite and in [0, 32767]");
        }
    }

    public double tuningVel(){
        return shooter.getVelocity();
    }
    public double tuningPower(){
        return shooter.getPower();
    }
    public double tuningPos(){
        return shooter.getTargetPosition();
    }
}
