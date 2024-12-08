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

                        commandFactory.driveToTarget(1300, -600, 0, .05, .5, 100),
                        commandFactory.driveToTarget(1300, -900, 0, .05, .5, 50),
                        commandFactory.driveToTarget(370, -900, 0, .05, .5, 100),
                        // sample 2
                        commandFactory.driveToTarget(1300, -900, 0, .05, .5, 100),
                        commandFactory.driveToTarget(1300, -1100, 0, .05, .5, 50),
                        commandFactory.driveToTarget(370, -1100, 0, .05, .5, 100),
                        // sample 3
                        commandFactory.driveToTarget(1300, -1100, 0, .05, .5, 100),
                        commandFactory.driveToTarget(1300, -1270, 0, .05, .5, 50),
                        commandFactory.driveToTarget(370, -1270, 0, .05, .5, 100),

                        commandFactory.driveToTarget(600, -1000, 0, .05, .5, 100),


                        commandFactory.pivotToStart()

                        );
    }
}
