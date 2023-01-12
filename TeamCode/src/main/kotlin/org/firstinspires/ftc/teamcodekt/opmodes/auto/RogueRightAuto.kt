package org.firstinspires.ftc.teamcodekt.opmodes.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import ftc.rogue.blacksmith.Anvil
import ftc.rogue.blacksmith.Scheduler
import ftc.rogue.blacksmith.units.GlobalUnits
import ftc.rogue.blacksmith.util.kt.pow
import org.firstinspires.ftc.teamcode.AutoData
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence

@Autonomous
class RogueRightAuto : RogueBaseAuto() {
    override fun executeOrder66() {
        val startPose = GlobalUnits.pos(91, -159, 90)
        val startTraj = mainTrajRefactor(startPose).build<TrajectorySequence>()

        bot.drive.poseEstimate = startPose

        waitForStart()

        bot.drive.followTrajectorySequenceAsync(startTraj)

        while (opModeIsActive() && !isStopRequested) {
            updateComponents()
        }
    }

    private fun mainTrajRefactor(startPose: Pose2d) =
        Anvil.formTrajectory(bot.drive, startPose)
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
            .resetBot()

    private fun Anvil.initialDepositPrep() = this
        .addTemporalMarker {
            bot.lift.goToHigh()
            bot.claw.close()
            bot.arm.setToForwardsPos()
            bot.wrist.setToForwardsPos()
        }

    private fun Anvil.awaitInitialGoToDeposit() = this
        .splineToSplineHeading(82.8, -11.0, 130.5, 117.5)

    private fun Anvil.awaitGoToDeposit(it: Int) = this
        .splineToSplineHeading(82.95 + (it * .525), -12 - (it * 3.975), 131.5 + (it * 2.5), 155.0)

    private fun Anvil.awaitGoToIntake(it: Int) =
        inReverse {
            splineTo(163.5 - (it * .775), -25.75 - (it * 1.05), 0.0)
        }

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

            bot.arm.setToBackwardsPosButLikeSliiiightlyHigher()
            bot.wrist.setToBackwardsPos()

            bot.claw.openForIntakeWide()
        }

    private fun Anvil.fastIntakePrep(iterations: Int) = this
        .addTemporalMarker {
            bot.lift.height  = liftOffsets[iterations]

            bot.arm.setToBackwardsPosButLikeSliiiightlyHigher()
            bot.wrist.setToBackwardsPos()

            bot.claw.openForIntakeNarrow()
            bot.intake.enable()
        }


    private fun Anvil.awaitRegularIntake() = this
        .addTemporalMarker(-75) {
            bot.intake.disable()
            bot.claw.close()
        }

        .addTemporalMarker(300) {
            bot.lift.goToHigh()
        }

        .addTemporalMarker(450) {
            bot.arm.setToForwardsPos()
            bot.wrist.setToForwardsPos()
        }

        .waitTime(325)

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
        }
        .waitTime(1000)
}
