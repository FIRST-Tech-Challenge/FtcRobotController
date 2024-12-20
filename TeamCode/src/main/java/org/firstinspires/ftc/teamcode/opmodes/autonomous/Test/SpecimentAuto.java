package org.firstinspires.ftc.teamcode.opmodes.autonomous.Test;

import com.arcrobotics.ftclib.command.Command;
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
                             commandFactory.driveToTarget(740, 200, 0, .05, .6, 30),
                                commandFactory.pivotToSpecimenDelivery(),
                             commandFactory.elbowToSpecimenPosition(),
                                commandFactory.extendSliderToSpecimen()
                        ),

                        commandFactory.extendSliderToDeliverSpecimen(),

                        new ParallelCommandGroup(
                                commandFactory.collapseSlider(),
                                commandFactory.driveToTarget(350, 0, 0, .05, .6, 30)
                        ),


                        commandFactory.driveToTarget(700, -600, 0, .05, 1, 100),
                        //push sample 1 to human player

                        commandFactory.driveToTarget(1300, -600, 0, .05, .5, 100),
                        commandFactory.driveToTarget(1300, -900, 0, .05, .5, 50),
                        commandFactory.driveToTarget(320, -900, 0, .05, .5, 100),
                        // sample 2
                        commandFactory.driveToTarget(1300, -900, 0, .05, .5, 100),
                        commandFactory.driveToTarget(1300, -1100, 0, .05, .5, 50),
                        commandFactory.driveToTarget(320, -1100, 0, .05, .5, 100),

                        commandFactory.driveToTarget(600, -1000, 180, .05, .7, 30),
                        commandFactory.driveToTarget(400, -1000, 180, .05, .4, 30),

                        commandFactory.pivotToSpecimenInTake(),

                        commandFactory.intakeFromWall(),


                        new ParallelCommandGroup(
                                commandFactory.driveToTarget(740, 270, 0, .05, .7, 30, 4000),
                                commandFactory.pivotToSpecimenDelivery(),
                                commandFactory.elbowToSpecimenPosition(),
                                commandFactory.extendSliderToSpecimen()
                        ),

                        commandFactory.extendSliderToDeliverSpecimen(),

                        new ParallelCommandGroup(
                                commandFactory.collapseSlider(),
                                commandFactory.driveToTarget(600, -1000, 180, .05, .7, 30)
                        ),

                        new ParallelCommandGroup(
                            commandFactory.driveToTarget(400, -1000, 180, .05, .7, 30),
                            commandFactory.pivotToSpecimenInTake()
                        ),

                        commandFactory.intakeFromWall(),

                        new ParallelCommandGroup(
                                commandFactory.driveToTarget(740, 130, 0, .05, .7, 30, 4000),
                                commandFactory.pivotToSpecimenDelivery(),
                                commandFactory.elbowToSpecimenPosition(),
                                commandFactory.extendSliderToSpecimen()
                        ),

                        commandFactory.extendSliderToDeliverSpecimen(),

                        commandFactory.pivotToStart()

                );
    }
}
