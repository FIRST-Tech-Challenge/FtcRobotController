package org.firstinspires.ftc.teamcodekt.opmodes.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import ftc.rogue.blacksmith.Anvil
import ftc.rogue.blacksmith.Scheduler
import ftc.rogue.blacksmith.units.GlobalUnits
import org.firstinspires.ftc.teamcode.AutoData
import kotlin.properties.Delegates

@Autonomous
class RogueLeftAuto : RogueBaseAuto() {
    override val startPose = GlobalUnits.pos(-91, -159, 90)

    override fun mainTraj(startPose: Pose2d) =
        Anvil.formTrajectory(bot.drive, startPose)
            .preform(0, ::parkTraj)

            .initialDepositPrep()

            .awaitInitialGoToDeposit()

            .awaitDeposit()

            .doTimes(NUM_CYCLES) {
                when (it) {
                    LAST_CYCLE -> fastIntakePrep(it)
                    else -> regularIntakePrep(it)
                }

                awaitGoToIntake(it)

                when (it) {
                    LAST_CYCLE -> awaitFastIntake()
                    else -> awaitRegularIntake()
                }

                awaitGoToDeposit(it)

                awaitDeposit()
            }
            .thenRunPreformed(0)

    private fun Anvil.initialDepositPrep() = this
        .addTemporalMarker {
            bot.lift.goToHigh()
            bot.claw.close()
            bot.arm.setToForwardsPos()
            bot.wrist.setToForwardsPos()
        }

    private fun Anvil.awaitInitialGoToDeposit() = this
        .splineToSplineHeading(-82.5, -12.75, 48.25, 65)

    private fun Anvil.awaitGoToDeposit(it: Int) = when (it) {
        0 -> splineToSplineHeading(-85.100, -10.250, 39.000, 25)
        1 -> splineToSplineHeading(-81.800, -14.905, 48.275, 25)
        2 -> splineToSplineHeading(-77.150, -16.450, 48.900, 25)
        3 -> splineToSplineHeading(-74.624, -19.575, 51.625, 25)
//        4 -> splineToSplineHeading(-74.950, -17.950, 39.600, 25)
        else -> this
    }

    private fun Anvil.awaitGoToIntake(it: Int) = when (it) {
        0 -> splineTo(-161.0000, -25.850, 180)
        1 -> splineTo(-160.2375, -27.175, 180)
        2 -> splineTo(-158.9100, -28.400, 180)
        3 -> splineTo(-156.5975, -29.675, 180)
//        4 -> splineTo(-154.6500, -30.950, 180)
        else -> noop
    }.doInReverse()

    private fun Anvil.awaitDeposit() = this
        .addTemporalMarker(-65) {
            bot.lift.height -= AutoData.DEPOSIT_DROP_AMOUNT
        }

        .addTemporalMarker {
            bot.claw.openForDeposit()
        }

        .waitTime(100)

    private fun Anvil.regularIntakePrep(iterations: Int) = this
        .addTemporalMarker {
            bot.lift.height = liftOffsets[iterations]
            bot.wrist.setToBackwardsPos()
            bot.arm.setToBackwardsPosButLikeSliiiightlyHigher()
        }

        .addTemporalMarker(150) {
            bot.claw.openForIntakeWide()
        }

    private fun Anvil.fastIntakePrep(iterations: Int) = this
        .addTemporalMarker {
            bot.lift.height  = liftOffsets[iterations]

            bot.arm.setToBackwardsPos()
            bot.wrist.setToBackwardsPos()

            bot.claw.openForIntakeNarrow()
            bot.intake.enable()
        }
    private fun Anvil.awaitRegularIntake() = this
        .addTemporalMarker {
            bot.intake.disable()
            bot.claw.close()
        }

        .addTemporalMarker(275) {
            bot.lift.goToHigh()
        }

        .addTemporalMarker(425) {
            bot.arm.setToForwardsPos()
            bot.wrist.setToForwardsPos()
        }

        .waitTime(300)

    private fun Anvil.awaitFastIntake() = this
        .addTemporalMarker(-75) {
            bot.intake.disable()
            bot.claw.close()
        }

        .addTemporalMarker(135) {
            bot.arm.setToForwardsPos()
            bot.lift.goToHigh()
        }

        .addTemporalMarker(250) {
            bot.wrist.setToForwardsPos()
        }

        .waitTime(120)

    private fun Anvil.resetBot() = this
        .addTemporalMarker {
            bot.arm.setToRestingPos()
            bot.wrist.setToRestingPos()
            bot.lift.goToZero()
            bot.claw.close()
        }


    private fun parkTraj(startPose: Pose2d) =
        Anvil.formTrajectory(bot.drive, startPose) {
            resetBot()

            when (signalID) {
                1 -> inReverse {
                    splineTo(-89, -33, -90)
                }
                2 -> inReverse {
                    splineTo(-144.65, -30.95, 180)
                }
                3 -> {
                    splineToLinearHeading(-30, -34, 90, -38.375)
                }
            }

            this
        }
}
