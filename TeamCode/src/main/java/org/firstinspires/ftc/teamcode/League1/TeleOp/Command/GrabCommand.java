package org.firstinspires.ftc.teamcode.League1.TeleOp.Command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.League1.Common.Constants;
import org.firstinspires.ftc.teamcode.League1.Common.Robot;
import org.firstinspires.ftc.teamcode.League1.Subsystems.ScoringSystem;

public class GrabCommand extends CommandBase {
    ScoringSystem score;
    private boolean close;


    public GrabCommand(ScoringSystem score, Constants constants, HardwareMap hardwareMap, Robot robot, boolean close){
        score = new ScoringSystem(hardwareMap, robot, constants, false);
        this.close = close;



    }

    @Override
    public void initialize() {
        score.grabberOpenAndClose(close);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
