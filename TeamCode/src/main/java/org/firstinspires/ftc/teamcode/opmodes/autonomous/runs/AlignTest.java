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

                double deliveryDistance = 87;
                double minDelivery = 70;
                double intakeDistance = 60;
                double minIntake = 45;

                return new SequentialCommandGroup(
                        // FIRST specimen

                        // Reach submersible
                        new ParallelDeadlineGroup(
                                commandFactory.checkForwardDistance(deliveryDistance, minDelivery, 4700),
                                commandFactory.extendSpecimenSlider(6000),
                                new SequentialCommandGroup(
                                        commandFactory.driveToTarget(580, 0, 0, .05, .4, 30, 2000),
                                        commandFactory.sleep(200),
                                        commandFactory.alignToSpecimenDelivery(deliveryDistance, minDelivery, 1500)
                                )
                        ),

                        // Drop
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

                        commandFactory.driveToTarget(500, -1150, 180, .05, .8, 100, 2000),

                        commandFactory.driveToTarget(200, -1150, 180, .05, .8, 100),

                        // Intake SECOND Specimen
                        new ParallelDeadlineGroup(
                                commandFactory.checkForwardDistance(intakeDistance, minIntake,3000),
                                commandFactory.alignToSpecimenDelivery(intakeDistance, minIntake,2500)
                        ),
                        commandFactory.closeSpecimenClaw(),
                        commandFactory.sleep(500),

                        new ParallelRaceGroup(
                                commandFactory.driveToTarget(300, -1100, 180, .05, .9, 140, 1000),
                                commandFactory.extendSpecimenSlider(6000)
                        ),

                        new ParallelRaceGroup(
                                commandFactory.driveToTarget(600, -50, 0, .05, .9, 50, 3000),
                                commandFactory.extendSpecimenSlider(6000)
                        ),

                        new ParallelDeadlineGroup(
                                commandFactory.checkForwardDistance(deliveryDistance, minDelivery, 5000),
                                commandFactory.extendSpecimenSlider(6000),
                                new SequentialCommandGroup(
                                        commandFactory.sleep(500),
                                        commandFactory.alignToSpecimenDelivery(deliveryDistance, minDelivery,3000)
                                )
                        ),

                        commandFactory.collapseSpecimenSlider(500),
                        commandFactory.openSpecimenClaw(),

                        // Drive to observation zone
                        new ParallelCommandGroup(
                                commandFactory.driveToTarget(200, -1200, 180, .05, .9, 100, 3000),
                                commandFactory.collapseSpecimenSlider(1000)
                        ),

                        new ParallelDeadlineGroup(
                                commandFactory.checkForwardDistance(intakeDistance, minIntake,5000),
                                commandFactory.alignToSpecimenDelivery(intakeDistance,minIntake,3000)
                        ),
                        commandFactory.closeSpecimenClaw(),
                        commandFactory.sleep(500),

                        new ParallelRaceGroup(
                                commandFactory.driveToTarget(300, -1100, 180, .05, .9, 150, 1000),
                                commandFactory.extendSpecimenSlider(6000)
                        ),

                        new ParallelRaceGroup(
                                commandFactory.driveToTarget(600, -100, 0, .05, .9, 150, 3000),
                                commandFactory.extendSpecimenSlider(6000)
                        ),

                        new ParallelDeadlineGroup(
                                commandFactory.checkForwardDistance(deliveryDistance, minDelivery,5000),
                                commandFactory.extendSpecimenSlider(6000),
                                new SequentialCommandGroup(
                                        commandFactory.sleep(500),
                                        commandFactory.alignToSpecimenDelivery(deliveryDistance, minDelivery, 3000)
                                )
                        ),

                        commandFactory.collapseSpecimenSlider(500),
                        commandFactory.openSpecimenClaw(),

                        commandFactory.driveToTarget(300, -1150, 0, .05, .8, 100, 2000),

                        commandFactory.sleep(10000)

                );
    }
}
