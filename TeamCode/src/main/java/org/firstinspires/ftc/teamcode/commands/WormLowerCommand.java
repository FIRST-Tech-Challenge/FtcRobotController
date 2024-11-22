package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Worm;

public class WormLowerCommand extends CommandBase {
    private Worm worm;
    private final double seconds;
    private int counter;
    private int target;

    public WormLowerCommand(Worm worm, double seconds) {
        this.worm = worm;
        this.seconds = seconds;

        // Convert time in seconds to robot cycles (50 cycles/s)
        target = (int)(seconds * 50);

        addRequirements(worm);
    }

    @Override
    public void execute() {
        if (counter < target)
            counter++;

        worm.lower(-0.3);
    }

    @Override
    public void end(boolean interrupted) {
        //need to set counter to zero after end or we can't keep running it
        counter = 0;
        worm.brake();
    }

    @Override
    public boolean isFinished() {
        return counter >= target;
    }
}
