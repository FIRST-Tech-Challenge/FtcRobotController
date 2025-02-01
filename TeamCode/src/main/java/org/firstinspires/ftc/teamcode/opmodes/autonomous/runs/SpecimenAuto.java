package org.firstinspires.ftc.teamcode.opmodes.autonomous.runs;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.base.CommandAutoOpMode;

@Autonomous
public class SpecimenAuto extends CommandAutoOpMode {

    @Override
    protected Command createCommand() {

        double deliveryDistance = 75;
        double minDelivery = 65;
        double intakeDistance = 60;
        double minIntake = 45;

        int firstY = -800;
        int secondY = -610 - 260;

        return new SequentialCommandGroup(
                // FIRST specimen

                // Reach submersible
                new ParallelDeadlineGroup(
                        commandFactory.checkForwardDistance(deliveryDistance, minDelivery, 4700),
                        commandFactory.extendSpecimenSlider(6000),
                        new SequentialCommandGroup(
                                commandFactory.driveToTarget(580, 0, 0, .05, .4, 30, 2000),
                                commandFactory.sleep(300),
                                commandFactory.alignToSpecimenDelivery(deliveryDistance, minDelivery, 1500)
                        )
                ),

                // Drop
                commandFactory.collapseSpecimenSlider(500),
                commandFactory.openSpecimenClaw(),

                //  Find first sample
                commandFactory.driveToTarget(330, 0, -25, .18, .7, 90, 2500),
                commandFactory.driveToTarget(340, firstY, -25, .18, .7, 90, 2500),
                new ParallelRaceGroup(
                        commandFactory.collapseSpecimenSlider(5000),
                        commandFactory.driveToTarget(365, firstY, -25, .18, .7, 30, 2500)
                ),

                new ParallelCommandGroup(
                        commandFactory.extendSliderToSweep(),
                        commandFactory.pivotToSweep(),
                        commandFactory.elbowToSweepPosition()
                ),

                commandFactory.sleep(200),

                // Turn to sweep
                commandFactory.driveToTarget(320, firstY, -120, .07, .45, 40, 1200),


                // Find next sample
                new ParallelCommandGroup(
                        commandFactory.pivotToJustAboveSweep(),
                        commandFactory.driveToTarget(580, secondY, -45, .18, .6, 30, 1500)
                ),

                commandFactory.pivotToSweep(),
                commandFactory.sleep(200),

                commandFactory.driveToTarget(350, secondY, -120, .07, .45, 40, 1200),



                new ParallelCommandGroup(
                        commandFactory.pivotToStart(),

                        new SequentialCommandGroup(
                                commandFactory.sleep(200),
                                new ParallelCommandGroup(
                                        commandFactory.collapseSlider(),
                                        commandFactory.elbowToStartPosition(),
                                        commandFactory.driveToTarget(200, -1100, -180, .05, .8, 100, 1500)
                                )
                        )
                ),

                PickupSpecimenAndDrop(intakeDistance,minIntake, deliveryDistance, minDelivery, -180, -359, -75),

                new ParallelCommandGroup(
                        commandFactory.collapseSpecimenSlider(1000),
                        commandFactory.driveToTarget(100, -1100, -180, .05, .9, 100, 3000)
                ),

                PickupSpecimenAndDrop(intakeDistance, minIntake, deliveryDistance, minDelivery, -180, -360, -125),

                new ParallelCommandGroup(
                        commandFactory.collapseSpecimenSlider(1000),
                        new SequentialCommandGroup(
                                commandFactory.driveToTarget(350, -250, -360, .05, 1, 2000, 750),
                                commandFactory.driveToTarget(50, -1100, -360, .05, 1, 100, 3000)
                        )
                ),

                commandFactory.sleep(10000)
        );
    }

    private SequentialCommandGroup PickupSpecimenAndDrop(
            double intakeDistance, double minIntake, double deliveryDistance, double minDelivery,
            double pickupHeading,
            double dropHeading,
            double dropY
    ) {
        return new SequentialCommandGroup(

                // Intake SECOND Specimen
                new ParallelDeadlineGroup(
                        commandFactory.checkForwardDistance(intakeDistance, minIntake,2000),
                        commandFactory.alignToSpecimenDelivery(intakeDistance, minIntake,2000)
                ),

                commandFactory.closeSpecimenClaw(),
                commandFactory.sleep(300),


                new ParallelRaceGroup(
                        commandFactory.driveToTarget(200, -1100, pickupHeading, .05, 1, 140, 500),
                        commandFactory.extendSpecimenSlider(6000)
                ),

                new ParallelRaceGroup(
                        commandFactory.driveToTarget(600, dropY, dropHeading, .07, 1, 70, 2300),
                        commandFactory.extendSpecimenSlider(6000)
                ),

                new ParallelDeadlineGroup(
                        commandFactory.checkForwardDistance(deliveryDistance, minDelivery, 5000),
                        commandFactory.extendSpecimenSlider(6000),
                        new SequentialCommandGroup(
                                commandFactory.sleep(200),
                                commandFactory.alignToSpecimenDelivery(deliveryDistance, minDelivery,3000)
                        )
                ),

                commandFactory.collapseSpecimenSlider(500),
                commandFactory.openSpecimenClaw()
        );
    }
}
