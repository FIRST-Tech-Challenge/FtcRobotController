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
        .splineToSplineHeading(-82.5, -12.75, 50, 65)

    private fun Anvil.awaitGoToDeposit(it: Int) = when (it) {
        0 -> splineToSplineHeading(-85.100, -10.250, 41.250, 25)
        1 -> splineToSplineHeading(-81.800, -14.905, 50.275, 25)
        2 -> splineToSplineHeading(-77.150, -13.450, 49.900, 25)
        3 -> splineToSplineHeading(-79.624, -12.575, 51.625, 25)
        4 -> splineToSplineHeading(-79.950, -12.950, 48.600, 25)
        else -> this
    }

    private fun Anvil.awaitGoToIntake(it: Int) = when (it) {
        0 -> splineTo(-163.9000, -21.250, 180)
        1 -> splineTo(-163.4375, -22.575, 180)
        2 -> splineTo(-162.9100, -21.400, 180)
        3 -> splineTo(-161.9975, -23.575, 180)
        4 -> splineTo(-160.9900, -23.750, 180)
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
            bot.lift.height = liftOffsets[iterations]

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

        .addTemporalMarker(15) {
            bot.arm.setToForwardsPos()
            bot.lift.goToHigh()
        }

        .addTemporalMarker(100) {
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
