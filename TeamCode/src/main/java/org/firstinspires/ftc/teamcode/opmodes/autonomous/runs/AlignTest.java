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

                double deliveryDistance = 75;
                double minDelivery = 65;
                double intakeDistance = 60;
                double minIntake = 45;

                int firstY = -700;
                int secondY = -580 - 230;

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


                        //  Find first sample
                        new ParallelRaceGroup(
                                commandFactory.collapseSpecimenSlider(5000),
                                commandFactory.driveToTarget(380, firstY, -30, .9, .8, 30, 1500)
                        ),

                        new ParallelCommandGroup(
                            commandFactory.extendSliderToSweep(),
                            commandFactory.pivotToSweep(),
                                commandFactory.elbowToSweepPosition()
                        ),

                        commandFactory.sleep(200),

                        // Turn to sweep
                        commandFactory.driveToTarget(320, firstY, -120, .05, .4, 30),


                        // Find next sample
                        new ParallelCommandGroup(
                                commandFactory.pivotToJustAboveSweep(),
                        commandFactory.driveToTarget(550, secondY, -45, .09, .6, 30, 1000)
                                ),

                        commandFactory.pivotToSweep(),
                        commandFactory.sleep(200),

                        commandFactory.driveToTarget(350, secondY, -120, .05, .4, 30),



                        new ParallelCommandGroup(
                                commandFactory.pivotToStart(),

                                new SequentialCommandGroup(
                                        commandFactory.sleep(200),
                                        new ParallelCommandGroup(
                                            commandFactory.collapseSlider(),
                                            commandFactory.elbowToDeliveryPosition(),
                                            commandFactory.driveToTarget(200, -1100, -180, .05, .8, 100, 1500)
                                        )
                                )
                        ),

                        PickupSpecimenAndDrop(intakeDistance,minIntake, deliveryDistance, minDelivery, -180),

                        new ParallelCommandGroup(
                                commandFactory.collapseSpecimenSlider(1000),
                                commandFactory.driveToTarget(100, -1100, 180, .05, .9, 100, 3000)
                        ),

                        PickupSpecimenAndDrop(intakeDistance,minIntake, deliveryDistance, minDelivery, 180),

                        new ParallelCommandGroup(
                                commandFactory.collapseSpecimenSlider(1000),
                                commandFactory.driveToTarget(100, -1100, 0, .05, 1, 100, 3000)
                        ),


//                        //push sample 1 to human player
//
//                        commandFactory.driveToTarget(1300, -950, 0, .05, .8, 100, 2000),
//                        commandFactory.driveToTarget(1300, -1150, 0, .05, .8, 100, 2000),
//                        commandFactory.driveToTarget(300, -1150, 0, .05, .8, 100, 2000),
//
//                        commandFactory.driveToTarget(500, -1150, 180, .05, .8, 100, 2000),
//
//                        commandFactory.driveToTarget(200, -1150, 180, .05, .8, 100),
//
//                        // Intake SECOND Specimen
//                        new ParallelDeadlineGroup(
//                                commandFactory.checkForwardDistance(intakeDistance, minIntake,3000),
//                                commandFactory.alignToSpecimenDelivery(intakeDistance, minIntake,2500)
//                        ),
//                        commandFactory.closeSpecimenClaw(),
//                        commandFactory.sleep(500),
//
//                        new ParallelRaceGroup(
//                                commandFactory.driveToTarget(300, -1100, 180, .05, .9, 140, 1000),
//                                commandFactory.extendSpecimenSlider(6000)
//                        ),
//
//                        new ParallelRaceGroup(
//                                commandFactory.driveToTarget(600, -50, 0, .05, .9, 50, 3000),
//                                commandFactory.extendSpecimenSlider(6000)
//                        ),
//
//                        new ParallelDeadlineGroup(
//                                commandFactory.checkForwardDistance(deliveryDistance, minDelivery, 5000),
//                                commandFactory.extendSpecimenSlider(6000),
//                                new SequentialCommandGroup(
//                                        commandFactory.sleep(500),
//                                        commandFactory.alignToSpecimenDelivery(deliveryDistance, minDelivery,3000)
//                                )
//                        ),
//
//                        commandFactory.collapseSpecimenSlider(500),
//                        commandFactory.openSpecimenClaw(),
//
//                        // Drive to observation zone
//                        new ParallelCommandGroup(
//                                commandFactory.driveToTarget(200, -1200, 180, .05, .9, 100, 3000),
//                                commandFactory.collapseSpecimenSlider(1000)
//                        ),
//
//                        new ParallelDeadlineGroup(
//                                commandFactory.checkForwardDistance(intakeDistance, minIntake,5000),
//                                commandFactory.alignToSpecimenDelivery(intakeDistance,minIntake,3000)
//                        ),
//                        commandFactory.closeSpecimenClaw(),
//                        commandFactory.sleep(500),
//
//                        new ParallelRaceGroup(
//                                commandFactory.driveToTarget(300, -1100, 180, .05, .9, 150, 1000),
//                                commandFactory.extendSpecimenSlider(6000)
//                        ),
//
//                        new ParallelRaceGroup(
//                                commandFactory.driveToTarget(600, -100, 0, .05, .9, 150, 3000),
//                                commandFactory.extendSpecimenSlider(6000)
//                        ),
//
//                        new ParallelDeadlineGroup(
//                                commandFactory.checkForwardDistance(deliveryDistance, minDelivery,5000),
//                                commandFactory.extendSpecimenSlider(6000),
//                                new SequentialCommandGroup(
//                                        commandFactory.sleep(500),
//                                        commandFactory.alignToSpecimenDelivery(deliveryDistance, minDelivery, 3000)
//                                )
//                        ),
//
//                        commandFactory.collapseSpecimenSlider(500),
//                        commandFactory.openSpecimenClaw(),

                        commandFactory.sleep(10000)

                );
    }

    private SequentialCommandGroup PickupSpecimenAndDrop(double intakeDistance, double minIntake, double deliveryDistance, double minDelivery, double pickupHeading) {
        return new SequentialCommandGroup(

        // Intake SECOND Specimen
        new ParallelDeadlineGroup(
                commandFactory.checkForwardDistance(intakeDistance, minIntake,2000),
                commandFactory.alignToSpecimenDelivery(intakeDistance, minIntake,2000)
        ),

                commandFactory.closeSpecimenClaw(),
                commandFactory.sleep(300),


                new ParallelRaceGroup(
                        commandFactory.driveToTarget(300, -1100, pickupHeading, .05, .9, 140, 1000),
                        commandFactory.extendSpecimenSlider(6000)
                ),

                new ParallelRaceGroup(
                        commandFactory.driveToTarget(600, -50, 0, .05, 1, 50, 1750),
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
                commandFactory.openSpecimenClaw()
        );
    }
}
