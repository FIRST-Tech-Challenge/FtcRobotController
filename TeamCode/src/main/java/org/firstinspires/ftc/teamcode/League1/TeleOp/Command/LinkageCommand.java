package org.firstinspires.ftc.teamcode.League1.TeleOp.Command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.League1.Common.Constants;
import org.firstinspires.ftc.teamcode.League1.Common.Robot;
import org.firstinspires.ftc.teamcode.League1.Subsystems.ScoringSystem;

public class LinkageCommand extends CommandBase {

    boolean up;
    ScoringSystem score;


    public LinkageCommand(ScoringSystem score, Constants constants, HardwareMap hardwareMap, Robot robot, boolean up){
        score = new ScoringSystem(hardwareMap, robot, constants, false);
        this.up = up;



    }

    @Override
    public void initialize() {
        score.linkageUpAndDown(up);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
