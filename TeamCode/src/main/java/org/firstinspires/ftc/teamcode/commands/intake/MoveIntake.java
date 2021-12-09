package org.firstinspires.ftc.teamcode.commands.intake;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.TriggerReader;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.carousel.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;

import java.util.function.DoubleSupplier;

public class MoveIntake extends CommandBase {

    private final IntakeSubsystem m_intakeSubsytem;

    private final double power;
    private final DoubleSupplier trigger;

    private Telemetry telemetry;


    public MoveIntake(IntakeSubsystem subsystem, double power, DoubleSupplier trigger){
        m_intakeSubsytem = subsystem;
        this.power = power;
        this.trigger = trigger;
        addRequirements(subsystem);
    }

    public MoveIntake(IntakeSubsystem subsystem, double power, DoubleSupplier trigger, Telemetry telemetry){
        m_intakeSubsytem = subsystem;
        this.power = power;
        this.trigger = trigger;
        this.telemetry = telemetry;

        addRequirements(subsystem);
    }


    @Override
    public void execute(){

        if(trigger.getAsDouble() > 0.5)
            m_intakeSubsytem.setPower(power);
    }

}
