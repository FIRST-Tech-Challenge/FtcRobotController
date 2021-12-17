package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.magnetic.limitswitch.MagneticLimitSwitchSubsystem;

import java.util.function.BooleanSupplier;

public class ResetArmCount extends CommandBase {

    public final MagneticLimitSwitchSubsystem magneticLimitSwitchSubsystem;
    public final ArmSubsystem armSubsytem;
    //private final BooleanSupplier magneticLimitSwitchPressed;
    private Telemetry telemetry;


    public ResetArmCount(MagneticLimitSwitchSubsystem subsystem, ArmSubsystem armSubsystem){
        magneticLimitSwitchSubsystem = subsystem;
        armSubsytem = armSubsystem;
        //magneticLimitSwitchPressed = isPressed;
        addRequirements(subsystem,armSubsystem);
    }

    public ResetArmCount(MagneticLimitSwitchSubsystem subsystem, ArmSubsystem armSubsystem, Telemetry telemetry){
        magneticLimitSwitchSubsystem = subsystem;
        armSubsytem = armSubsystem;
        //magneticLimitSwitchPressed = isPressed;
        this.telemetry = telemetry;
        addRequirements(subsystem, armSubsystem);
    }

    @Override
    public void initialize(){
        armSubsytem.setZero();
        telemetry.addData("reset arm encoders to zero","");
        telemetry.update();
    }

    @Override
    public void execute(){
        //if(magneticLimitSwitchPressed.getAsBoolean()){
            armSubsytem.setZero();
        //}
        //telemetry.addData("reset arm encoders to zero",magneticLimitSwitchPressed.getAsBoolean());
        //telemetry.update();

    }

    /*@Override
    public boolean isFinished(){
        return true;
    }*/

}
