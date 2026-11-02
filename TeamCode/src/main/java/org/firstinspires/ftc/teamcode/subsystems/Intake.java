package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.jetbrains.annotations.NotNull;

/** Owns the intake motor; the TeleOp decides when to collect. */
@Configurable
public final class Intake extends SubsystemBase {
    // Shared commanded power, limited to [-1, 1] by setPower(); tune in Panels.
    public static double INTAKE_POWER = 1.0;
    private final DcMotorEx intake;
    private double commandedPower;

    public Intake(@NotNull final HardwareMap hMap) {
        intake = hMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        stop();
    }

    public void runIntake() {
        setPower(INTAKE_POWER);
    }

    public void setPower(double power) {
        double limitedPower = Math.max(-1.0, Math.min(1.0, power));
        intake.setPower(limitedPower);
        commandedPower = limitedPower;
    }

    public void stop() {
        setPower(0.0);
    }

    /** Last successfully written command, not a measurement of mechanism motion. */
    public double getCommandedPower() {
        return commandedPower;
    }

    public double tuningPow() { return intake.getPower(); }
}
