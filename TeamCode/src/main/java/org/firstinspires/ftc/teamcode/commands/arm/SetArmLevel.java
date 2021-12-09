package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem;

public class SetArmLevel extends CommandBase {

    private final ArmSubsystem m_armSubsytem;

    private final Integer levelIndicator;

    private Telemetry telemetry;


    public SetArmLevel(ArmSubsystem subsystem, Integer levelIndicator){
        m_armSubsytem = subsystem;
        this.levelIndicator = levelIndicator;
        addRequirements(subsystem);
    }

    public SetArmLevel(ArmSubsystem subsystem, Integer levelIndicator, Telemetry telemetry){
        m_armSubsytem = subsystem;
        this.levelIndicator = levelIndicator;
        this.telemetry = telemetry;

        addRequirements(subsystem);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        m_armSubsytem.setArmTargetPosition(m_armSubsytem.getLevel(levelIndicator));
    }

}
