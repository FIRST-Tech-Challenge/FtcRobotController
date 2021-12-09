package org.firstinspires.ftc.teamcode.commands.intake;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.TriggerReader;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.carousel.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;

public class MoveIntake extends CommandBase {

    private final IntakeSubsystem m_intakeSubsytem;

    private final TriggerReader reader;

    private final Double power;

    private Telemetry telemetry;


    public MoveIntake(IntakeSubsystem subsystem, Double power, TriggerReader reader){
        m_intakeSubsytem = subsystem;
        this.power = power;
        this.reader = reader;
        addRequirements(subsystem);
    }

    public MoveIntake(IntakeSubsystem subsystem, Double power, TriggerReader reader, Telemetry telemetry){
        m_intakeSubsytem = subsystem;
        this.power = power;
        this.reader = reader;
        this.telemetry = telemetry;

        addRequirements(subsystem);
    }


    @Override
    public void initialize(){

    }


    @Override
    public void execute(){

        if(reader.isDown()){
            m_intakeSubsytem.setPower(power);
        }
    }

}
