package org.firstinspires.ftc.teamcode.opmodes.autonomous.Test;

import android.util.Log;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.base.CommandAutoOpMode;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.command.SleeperCommand;

//@Autonomous
public class TimeoutTest extends CommandAutoOpMode {
    @Override
    protected Command createCommand() {
        return new SequentialCommandGroup(
                new SleeperCommand(5000),
                new InstantCommand(() -> Log.i("TimeoutTest",  "Command after long sleep executed"))
        );
    }
}
