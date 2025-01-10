package org.firstinspires.ftc.teamcode.opmodes.autonomous.runs;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.base.CommandAutoOpMode;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.command.AlignDriveTrainToSpecimenDelivery;

@Autonomous
public class AlignTest extends CommandAutoOpMode {

    @Override
    protected Command createCommand() {
                return new SequentialCommandGroup(

                        // FIRST specimen

                        // Reach submersible
                        new ParallelDeadlineGroup(
                                commandFactory.checkForwardDistance(90,5000),
                                commandFactory.extendSpecimenSlider(6000),
                                new SequentialCommandGroup(
                                        commandFactory.driveToTarget(600, 0, 0, .05, .4, 30, 2500),
                                        commandFactory.sleep(200),
                                        commandFactory.alignToSpecimenDelivery(90,2000)
                                )
                        ),

                        // Drop
                        commandFactory.collapseSpecimenSlider(500),
                        commandFactory.openSpecimenClaw(),

                        // Drive to observation zone
                        new ParallelCommandGroup(
                            commandFactory.driveToTarget(200, -1200, 180, .05, .9, 100, 3000),
                                commandFactory.collapseSpecimenSlider(1000)
                        ),

                        // Intake
                        new ParallelDeadlineGroup(
                                commandFactory.checkForwardDistance(30,5000),
                                commandFactory.alignToSpecimenDelivery(30,3000)
                        ),
                        commandFactory.closeSpecimenClaw(),
                        commandFactory.sleep(500),

                        new ParallelRaceGroup(
                                commandFactory.driveToTarget(300, -1100, 180, .05, .9, 100, 1000),
                                commandFactory.extendSpecimenSlider(6000)
                        ),

                        new ParallelRaceGroup(
                                commandFactory.driveToTarget(600, -50, 0, .05, .9, 100, 3000),
                                commandFactory.extendSpecimenSlider(6000)
                        ),

                        new ParallelDeadlineGroup(
                                commandFactory.checkForwardDistance(90,5000),
                                commandFactory.extendSpecimenSlider(6000),
                                new SequentialCommandGroup(
                                        commandFactory.sleep(500),
                                        commandFactory.alignToSpecimenDelivery(90,3000)
                                )
                        ),

                        commandFactory.collapseSpecimenSlider(500),
                        commandFactory.openSpecimenClaw(),

                        new ParallelRaceGroup(
                                commandFactory.collapseSpecimenSlider(5000),
                                commandFactory.driveToTarget(300, -950, 0, .05, .8, 100)
                        ),

                        //push sample 1 to human player

                        commandFactory.driveToTarget(1300, -950, 0, .05, .8, 100, 2000),
                        commandFactory.driveToTarget(1300, -1150, 0, .05, .8, 100, 2000),
                        commandFactory.driveToTarget(300, -1150, 0, .05, .8, 100, 2000),

                        commandFactory.driveToTarget(450, -1150, 180, .05, .8, 100, 2000),

                        // Wait for human player to add clip
                        commandFactory.sleep(2000),

                        commandFactory.driveToTarget(300, -1150, 180, .05, .5, 100),

                        new ParallelDeadlineGroup(
                                commandFactory.checkForwardDistance(30,5000),
                                commandFactory.alignToSpecimenDelivery(30,3000)
                        ),
                        commandFactory.closeSpecimenClaw(),
                        commandFactory.sleep(500),

                        new ParallelRaceGroup(
                                commandFactory.driveToTarget(300, -1100, 180, .05, .9, 100, 1000),
                                commandFactory.extendSpecimenSlider(6000)
                        ),

                        new ParallelRaceGroup(
                                commandFactory.driveToTarget(600, -50, 0, .05, .9, 30, 3000),
                                commandFactory.extendSpecimenSlider(6000)
                        ),

                        new ParallelDeadlineGroup(
                                commandFactory.checkForwardDistance(90,5000),
                                commandFactory.extendSpecimenSlider(6000),
                                new SequentialCommandGroup(
                                        commandFactory.sleep(500),
                                        commandFactory.alignToSpecimenDelivery(90,3000)
                                )
                        ),

                        commandFactory.collapseSpecimenSlider(500),
                        commandFactory.openSpecimenClaw(),

                        commandFactory.sleep(10000)

                );
    }
}
