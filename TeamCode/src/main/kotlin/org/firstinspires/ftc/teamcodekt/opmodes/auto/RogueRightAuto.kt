package org.firstinspires.ftc.teamcodekt.opmodes.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import ftc.rogue.blacksmith.Anvil
import ftc.rogue.blacksmith.Scheduler
import ftc.rogue.blacksmith.units.GlobalUnits
import org.firstinspires.ftc.teamcode.AutoData
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcodekt.components.ClawConfig
import kotlin.properties.Delegates

@Autonomous
class RogueRightAuto : RogueBaseAuto() {
    private var signalID by Delegates.notNull<Int>()

    override fun execute() {
        val startPose = GlobalUnits.pos(91, -159, 90)
        val startTraj = mainTraj(startPose)

        Anvil.startAutoWith(startTraj).onSchedulerLaunch()

        bot.camera.update()
        signalID = bot.camera.waitForStartWithVision(this)

        Scheduler.launch(this, ::updateComponents)
    }

    private fun mainTraj(startPose: Pose2d) =
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
            bot.arm.setToForwardsPosAuto()
            bot.wrist.setToForwardsPos()
        }

    private fun Anvil.awaitInitialGoToDeposit() = this
        .splineToSplineHeading(82.5, -12.75, 131.5, 115.0)

    private fun Anvil.awaitGoToDeposit(it: Int) = when(it) {
        0 -> splineToSplineHeading(85.100, -10.250, 140.500, 155.0)
        1 -> splineToSplineHeading(81.800, -14.905, 133.825, 155.0)
        2 -> splineToSplineHeading(78.150, -17.450, 130.100, 155.0)
        3 -> splineToSplineHeading(75.624, -20.575, 127.375, 155.0)
        4 -> splineToSplineHeading(75.650, -16.250, 140.400, 155.0)
        else -> this
    }

    private fun Anvil.awaitGoToIntake(it: Int) =
        inReverse {
            when(it) {
                // TODO: Change intake Y. Knocking over the cone stack sometimes as of before LM3 match 3.
                0 -> splineTo(163.5500 - 1.85, -25.750, 0.0)
                1 -> splineTo(161.7375 - 1.5, -26.925, 0.0)
                2 -> splineTo(159.9250 - 1.015, -28.100, 0.0)
                3 -> splineTo(158.1125 - 1.515, -29.275, 0.0)
                4 -> splineTo(156.3000 - 1.65, -30.450, 0.0)
            }
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
            bot.wrist.setToBackwardsPos()
//            bot.arm.targetAngle = 58.5
            bot.arm.setToBackwardsPosButLikeSliiiightlyHigher()
        }

        .addTemporalMarker(150) {
//            bot.claw.targetPos = .5755
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
            bot.arm.setToForwardsPosAuto()
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

            back(15.0)

            telemetry.addData("Signal ID", signalID)

            when (signalID) {
                1 -> {
                    turn(38.0)
                    forward(61.0)
                }
                3 -> {
                    turn(45.0)
                    back(60.0)
                }
                else -> {
                    turn(35.0)
                    back(5.0)
                }
            }

            this
        }
}
