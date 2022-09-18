package org.firstinspires.ftc.teamcode.League1.TeleOp.Command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.League1.Common.Constants;
import org.firstinspires.ftc.teamcode.League1.Common.Robot;
import org.firstinspires.ftc.teamcode.League1.Subsystems.ScoringSystem;

public class LiftCommand extends CommandBase {
    ScoringSystem score;
    private ScoringSystem.ExtensionHeight position;
    private double power;




    public LiftCommand(ScoringSystem score, Constants constants, HardwareMap hardwareMap, Robot robot, ScoringSystem.ExtensionHeight height, double power){
        score = new ScoringSystem(hardwareMap, robot, constants, false);
        this.position = position;
        this.power = power;



    }

    @Override
    public void initialize() {

        //TODO: change this later so that it accepts enum
        //score.moveToPosition(position, power);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
