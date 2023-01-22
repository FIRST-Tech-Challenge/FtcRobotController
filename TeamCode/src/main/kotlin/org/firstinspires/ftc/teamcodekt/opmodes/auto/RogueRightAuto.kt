package org.firstinspires.ftc.teamcodekt.opmodes.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import ftc.rogue.blacksmith.Anvil
import ftc.rogue.blacksmith.units.GlobalUnits
import org.firstinspires.ftc.teamcode.AutoData

@Autonomous
class RogueRightAuto : RogueBaseAuto() {
    override val startPose = GlobalUnits.pos(91, -159, 90)

    override fun mainTraj(startPose: Pose2d) =
        Anvil.formTrajectory(bot.drive, startPose)
            .preform(0, ::parkTraj)

            .initialDepositPrep()

            .awaitInitialGoToDeposit()

            .deposit()

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

                deposit()
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
        .splineToSplineHeading(81, -11.35, 135.75, 115)

    private fun Anvil.awaitGoToDeposit(it: Int) = when (it) {
        0 -> splineToSplineHeading(81.000, -11.750, 142.200, 155)
        1 -> splineToSplineHeading(78.800, -13.905, 140.725, 155)
        2 -> splineToSplineHeading(76.850, -15.750, 137.850, 155)
        3 -> splineToSplineHeading(74.624, -19.575, 136.275, 155)
        4 -> splineToSplineHeading(75.950, -18.950, 145.900, 155)
        else -> this
    }

    private fun Anvil.awaitGoToIntake(it: Int) = when (it) {
        0 -> splineTo(161.0000, -25.850, 0)
        1 -> splineTo(160.2375, -27.175, 0)
        2 -> splineTo(158.9100, -28.400, 0)
        3 -> splineTo(156.5975, -29.675, 0)
        4 -> splineTo(154.6500, -30.950, 0)
        else -> noop
    }.doInReverse()

    private fun Anvil.deposit() = this
        .addTemporalMarker(-165) {
            bot.lift.height -= AutoData.DEPOSIT_DROP_AMOUNT
        }

        .addTemporalMarker(-100) {
            bot.claw.openForDeposit()
        }

    private fun Anvil.regularIntakePrep(iterations: Int) = this
        .addTemporalMarker(185) {
            bot.lift.height = liftOffsets[iterations]
            bot.wrist.setToBackwardsPos()
            bot.arm.setToBackwardsPosButLikeSliiiightlyHigher()
        }

        .addTemporalMarker(325) {
            bot.claw.openForIntakeWide()
        }

    private fun Anvil.fastIntakePrep(iterations: Int) = this
        .addTemporalMarker(185) {
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
                1 -> {
                    splineToLinearHeading(30, -34, 90, 218)
                }
                2 -> inReverse {
                    splineTo(145, -31, 0)
                }
                3 -> inReverse {
                    splineTo(89, -33, 270)
                }
            }

            this
        }
}
