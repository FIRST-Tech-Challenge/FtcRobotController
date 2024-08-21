package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.rustlib.commandsystem.PurePursuitAutonomousCommand;

@Autonomous(name = "Blue Right")
public class BlueRight extends AutonomousContainer {
    @Override
    public PurePursuitAutonomousCommand getAutonomousCommand() {
        return blueRightCommand;
    }
}
