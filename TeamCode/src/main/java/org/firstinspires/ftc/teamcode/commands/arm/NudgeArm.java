package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.cv.OpenCvShippingElementDetector;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.webcam.WebCamSubsystem;

import java.util.function.DoubleSupplier;

public class NudgeArm extends CommandBase {

    private final ArmSubsystem armSubsytem;

    private final int nudge;
    private int armSetPos = 0;
    //private final DoubleSupplier trigger;

    private Telemetry telemetry;


    public NudgeArm(ArmSubsystem subsystem, int nudge){
        armSubsytem = subsystem;
        this.nudge = nudge;

        addRequirements(armSubsytem);
    }

    public NudgeArm(ArmSubsystem subsystem, int nudge, Telemetry telemetry){
        armSubsytem = subsystem;
        this.nudge = nudge;

        this.telemetry = telemetry;

        addRequirements(armSubsytem);
    }

    @Override
    public void initialize(){
        int currentArmPos = armSubsytem.getCurrentPosition();
        telemetry.addData("nudigng in command", currentArmPos);
        armSetPos = currentArmPos + nudge;
        armSubsytem.setArmTargetPosition(armSetPos);
    }
    @Override
    public void execute(){


    }


}
