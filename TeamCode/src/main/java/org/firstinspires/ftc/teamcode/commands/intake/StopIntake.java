package org.firstinspires.ftc.teamcode.commands.intake;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.carousel.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;

public class StopIntake extends CommandBase {

    private final IntakeSubsystem m_intakeSubsytem;

    private final Double power = 0.0;

    private Telemetry telemetry;


    public StopIntake(IntakeSubsystem subsystem){
        m_intakeSubsytem = subsystem;
        addRequirements(subsystem);
    }

    public StopIntake(IntakeSubsystem subsystem, Telemetry telemetry){
        m_intakeSubsytem = subsystem;
        this.telemetry = telemetry;

        addRequirements(subsystem);
    }


    @Override
    public void initialize(){
        m_intakeSubsytem.setPower(power);
    }

}
