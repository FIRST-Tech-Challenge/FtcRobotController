package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem;

import java.util.function.DoubleSupplier;

public class NudgeArmWithSupplier extends CommandBase {

    private final ArmSubsystem m_armSubsytem;

    private final int nudge;
    private int armSetPos = 0;
    private final DoubleSupplier m_rs_y;

    private Telemetry telemetry;


    public NudgeArmWithSupplier(ArmSubsystem subsystem, DoubleSupplier rs_y, int nudge){
        m_armSubsytem = subsystem;
        m_rs_y = rs_y;
        this.nudge = nudge;

        addRequirements(subsystem);
    }

    public NudgeArmWithSupplier(ArmSubsystem subsystem, DoubleSupplier rs_y, int nudge, Telemetry telemetry){
        m_armSubsytem = subsystem;
        m_rs_y = rs_y;
        this.nudge = nudge;

        this.telemetry = telemetry;

        addRequirements(subsystem);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){

        double controller2rsticky = m_rs_y.getAsDouble();

        int currentArmPos = m_armSubsytem.getCurrentPosition();


        if(controller2rsticky == 1){
            armSetPos = currentArmPos - nudge;
        }
        else if(controller2rsticky == -1){
            armSetPos = currentArmPos + nudge;
        }


        telemetry.addData("nudigng in command", armSetPos);

        m_armSubsytem.setArmTargetPosition(armSetPos);
    }

}
