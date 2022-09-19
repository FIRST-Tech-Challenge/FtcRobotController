package org.firstinspires.ftc.teamcode.League1.TeleOp.CommandGroup;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.League1.Common.Constants;
import org.firstinspires.ftc.teamcode.League1.Common.Robot;
import org.firstinspires.ftc.teamcode.League1.Subsystems.ScoringSystem;
import org.firstinspires.ftc.teamcode.League1.TeleOp.Command.GrabCommand;
import org.firstinspires.ftc.teamcode.League1.TeleOp.Command.LiftCommand;
import org.firstinspires.ftc.teamcode.League1.TeleOp.Command.LinkageCommand;

public class ResetScoringGroup extends SequentialCommandGroup {


    public ResetScoringGroup(ScoringSystem score, Constants constants, HardwareMap hardwareMap, Robot robot){
        super(

                //Score
                new GrabCommand(score, constants, hardwareMap, robot, false),

                new WaitCommand(1000),


                //Reset
                new LiftCommand(score, constants, hardwareMap, robot, ScoringSystem.ExtensionHeight.ZERO, 0.5),

                new WaitCommand(500),

                new LinkageCommand(score, constants, hardwareMap, robot, false)
        );
    }
}
