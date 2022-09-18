package org.firstinspires.ftc.teamcode.League1.TeleOp.CommandGroup;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.League1.Common.Constants;
import org.firstinspires.ftc.teamcode.League1.Common.Robot;
import org.firstinspires.ftc.teamcode.League1.Subsystems.ScoringSystem;
import org.firstinspires.ftc.teamcode.League1.TeleOp.Command.GrabCommand;
import org.firstinspires.ftc.teamcode.League1.TeleOp.Command.LinkageUpCommand;

public class GrabAndLinkageUpGroup extends SequentialCommandGroup {


    public GrabAndLinkageUpGroup(ScoringSystem score, Constants constants, HardwareMap hardwareMap, Robot robot){
        super(
            new GrabCommand(score, constants, hardwareMap, robot, true),
            new WaitCommand(500),
            new LinkageUpCommand(score, constants, hardwareMap, robot, true)
        );
    }
}
