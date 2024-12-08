package org.firstinspires.ftc.teamcode.opmodes.autonomous.Test;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.base.CommandAutoOpMode;

@Autonomous
public class SpecimentAuto extends CommandAutoOpMode {

    boolean hold1End = false;
    boolean hold2End = false;
    boolean hold3End = false;
    @Override
    protected Command createCommand() {
                return new SequentialCommandGroup(
                        new ParallelCommandGroup(
                             commandFactory.driveToTarget(650, 200, 0, .05, .6, 30),
                                commandFactory.pivotToSpecimenDelivery(),
                             commandFactory.elbowToSpecimenPosition(),
                                commandFactory.extendSliderToSpeciment()
                        ),
                        commandFactory.extendSliderToDeliverSpeciman(),

                        new ParallelCommandGroup(
                                commandFactory.collapseSlider(),
                                commandFactory.driveToTarget(350, 0, 0, .05, .6, 30)
                        ),


                        commandFactory.driveToTarget(700, -600, 0, .05, 1, 100),
                        //push sample 1 to human player

                        commandFactory.driveToTarget(1100, -600, 0, .05, .7, 100),
                        commandFactory.driveToTarget(1100, -800, 0, .05, .7, 50),
                        commandFactory.driveToTarget(300, -800, 0, .05, .5, 100),

                        commandFactory.pivotToStart()

                        );
    }
}
