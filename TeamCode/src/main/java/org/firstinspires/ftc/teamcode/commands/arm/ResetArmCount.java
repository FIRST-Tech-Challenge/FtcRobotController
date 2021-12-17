package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem;

import java.util.function.BooleanSupplier;

public class ResetArmCount extends CommandBase {

    public final ArmSubsystem armSubsytem;
    private final BooleanSupplier magneticLimitSwitchPressed;
    private Telemetry telemetry;


    public ResetArmCount(ArmSubsystem subsystem, BooleanSupplier isPressed){
        armSubsytem = subsystem;
        magneticLimitSwitchPressed = isPressed;
        addRequirements(subsystem);
    }

    public ResetArmCount(ArmSubsystem subsystem, BooleanSupplier isPressed, Telemetry telemetry){
        armSubsytem = subsystem;
        magneticLimitSwitchPressed = isPressed;
        this.telemetry = telemetry;
        addRequirements(subsystem);
    }

    @Override
    public void initialize(){




    }

    @Override
    public void execute(){
        if(magneticLimitSwitchPressed.getAsBoolean()){
            armSubsytem.setZero();
        }
        telemetry.addData("reset arm encoders to zero","");
        telemetry.update();

    }

    /*@Override
    public boolean isFinished(){
        return true;
    }*/

}
