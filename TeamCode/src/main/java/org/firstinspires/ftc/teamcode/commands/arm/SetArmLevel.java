package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem;

public class SetArmLevel extends CommandBase {

    private final ArmSubsystem armSubsytem;

    private final Integer levelIndicator;

    private Telemetry telemetry;


    public SetArmLevel(ArmSubsystem subsystem, Integer levelIndicator){
        armSubsytem = subsystem;
        this.levelIndicator = levelIndicator;
        addRequirements(subsystem);
    }

    public SetArmLevel(ArmSubsystem subsystem, Integer levelIndicator, Telemetry telemetry){
        armSubsytem = subsystem;
        telemetry.addData("SetArmLevel",levelIndicator);
        telemetry.update();
        this.levelIndicator = levelIndicator;
        this.telemetry = telemetry;

        addRequirements(subsystem);
    }


    @Override
    public void initialize(){

        telemetry.addData("moving arm command",armSubsytem.getLevel(levelIndicator));
        telemetry.update();
        armSubsytem.setArmTargetPosition(armSubsytem.getLevel(levelIndicator));

    }

    @Override
    public void execute(){

    }

    @Override
    public boolean isFinished(){
        return true;
    }

}
