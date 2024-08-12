package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.rustlib.commandsystem.PurePursuitAutonomousCommand;

@Autonomous(name = "Red Left")
public class RedLeft extends AutonomousContainer {
    @Override
    public PurePursuitAutonomousCommand getAutonomousCommand() {
        return redLeftCommand;
    }
}
