package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem;

public class ResetArmCount extends CommandBase {

    public final ArmSubsystem armSubsytem;
    //private final BooleanSupplier magneticLimitSwitchPressed;
    private Telemetry telemetry;


    public ResetArmCount(ArmSubsystem subsystem){
        armSubsytem = subsystem;
        addRequirements(subsystem);
    }

    public ResetArmCount(ArmSubsystem subsystem, Telemetry telemetry){

        armSubsytem = subsystem;

        this.telemetry = telemetry;
        addRequirements(subsystem);
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
            //armSubsytem.setZero();
        //}
        //telemetry.addData("reset arm encoders to zero",magneticLimitSwitchPressed.getAsBoolean());
        //telemetry.update();

    }

    /*@Override
    public boolean isFinished(){
        return true;
    }*/

}
