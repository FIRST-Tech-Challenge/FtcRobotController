package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem;

public class NudgeArmWithStick extends CommandBase {

    private final ArmSubsystem m_armSubsytem;

    private final int nudge;
    private int armSetPos = 0;
    //private final DoubleSupplier trigger;

    private Telemetry telemetry;


    public NudgeArmWithStick(ArmSubsystem subsystem, int nudge){
        m_armSubsytem = subsystem;
        this.nudge = nudge;

        addRequirements(subsystem);
    }

    public NudgeArmWithStick(ArmSubsystem subsystem, int nudge, Telemetry telemetry){
        m_armSubsytem = subsystem;
        this.nudge = nudge;

        this.telemetry = telemetry;

        addRequirements(subsystem);
    }

    @Override
    public void initialize(){
        int currentArmPos = m_armSubsytem.getCurrentPosition();
        telemetry.addData("nudigng in command", currentArmPos);

        armSetPos = currentArmPos + nudge;

        m_armSubsytem.setArmTargetPosition(armSetPos);
    }


}
