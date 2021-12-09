package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.cv.OpenCvShippingElementDetector;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.webcam.WebCamSubsystem;

public class NudgeArm extends CommandBase {

    private final ArmSubsystem m_armSubsytem;

    private final int nudge;

    private Telemetry telemetry;


    public NudgeArm(ArmSubsystem subsystem, int nudge){
        m_armSubsytem = subsystem;
        this.nudge = nudge;
        addRequirements(subsystem);
    }

    public NudgeArm(ArmSubsystem subsystem, int nudge, Telemetry telemetry){
        m_armSubsytem = subsystem;
        this.nudge = nudge;
        this.telemetry = telemetry;

        addRequirements(subsystem);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        int currentArmPos = m_armSubsytem.getCurrentPosition();
        currentArmPos = currentArmPos + nudge;
        m_armSubsytem.setArmTargetPosition(currentArmPos);
    }

}
