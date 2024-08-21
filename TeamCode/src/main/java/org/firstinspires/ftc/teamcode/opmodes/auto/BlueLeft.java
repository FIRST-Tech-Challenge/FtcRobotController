package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.rustlib.commandsystem.PurePursuitAutonomousCommand;

@Autonomous(name = "Blue Left")
public class BlueLeft extends AutonomousContainer {
    @Override
    public PurePursuitAutonomousCommand getAutonomousCommand() {
        return blueLeftCommand;
    }
}
