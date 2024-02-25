package org.firstinspires.ftc.teamcode.CenterStageRobot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.IntakeSubsystem;
import org.inventors.ftc.robotbase.hardware.MotorExEx;

import java.util.concurrent.TimeUnit;
import java.util.function.DoubleSupplier;

public class IntakeManualCommand extends CommandBase {
    private IntakeSubsystem intakeSubsystem;
    private DoubleSupplier power;

    public IntakeManualCommand(IntakeSubsystem intakeSubsystem, DoubleSupplier power) {
        this.intakeSubsystem = intakeSubsystem;
        this.power = power;

        addRequirements(this.intakeSubsystem);
    }

    @Override
    public void execute() {
        intakeSubsystem.setPower(power.getAsDouble());
    }
}
