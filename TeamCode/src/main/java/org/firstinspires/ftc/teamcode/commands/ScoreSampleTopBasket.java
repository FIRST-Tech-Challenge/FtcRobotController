package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.Grabber;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Worm;

public class ScoreSampleTopBasket extends CommandBase {

    private Grabber grabber;
    private Wrist wrist;
    private Elevator elevator;
    private Worm worm;

    public ScoreSampleTopBasket(Grabber grabber, Wrist wrist, Elevator elevator, Worm worm) {
        this.grabber = grabber;
        this.wrist = wrist;
        this.elevator = elevator;
        this.worm = worm;
        addRequirements(grabber, wrist, elevator, worm);
    }

    @Override
    public void execute() {
        SequentialCommandGroup score = new SequentialCommandGroup(
                new RunCommand(worm::scoreBasket, worm).withTimeout(2000),
                new RunCommand(wrist::scoreBasket, wrist).withTimeout(1000),
                new RunCommand(elevator::scoreBasket, elevator).withTimeout(2000),
                new RunCommand(grabber::scoreBasket, grabber).withTimeout(1000)
        );

        score.execute();
    }
    @Override
    public void initialize() {
        grabber.pickup();
    }

    @Override
    public void end(boolean interrupted) {
        grabber.stop();
    }
}
