package org.firstinspires.ftc.teamcodekt.opmodes.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import ftc.rogue.blacksmith.Anvil
import ftc.rogue.blacksmith.units.DistanceUnit
import ftc.rogue.blacksmith.units.GlobalUnits
import ftc.rogue.blacksmith.util.toCm
import org.firstinspires.ftc.teamcode.AutoData
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcodekt.util.CycleException

@Autonomous
class RogueMidLeftAuto : RogueBaseAuto() {
    override val startPose = GlobalUnits.pos(-91, -163, 90)

    override fun mainTraj(startPose: Pose2d) =
        Anvil.forgeTrajectory(bot.drive, startPose)
            .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40.0, Math.toRadians(250.0), DriveConstants.TRACK_WIDTH))

            .addTemporalMarker {
                bot.lift.goToAngledMidPredeposit()
                bot.claw.close()
                bot.arm.setToForwardsAngledPos()
                bot.wrist.setToForwardsPos()
            }

            .initialGoToDeposit()
            .initialDeposit()

            .doTimes(NUM_CYCLES) {
                when (it) {
                    LAST_CYCLE -> fastIntakePrep(it)
                    else -> regularIntakePrep(it)
                }

                goToIntake(it)

                when (it) {
                    LAST_CYCLE -> awaitFastIntake()
                    else -> awaitRegularIntake()
                }

                goToDeposit(it)
                deposit(it)
            }

            .thenRun(::parkTraj)

    private fun Anvil.initialGoToDeposit() = this
        .forward(132)
        .turn(-139.5)
        .lineToLinearHeading(-78.75 + poleOffset.x, -42.5 + poleOffset.y, -49)

    private fun Anvil.goToDeposit(it: Int) = when (it) {
        0 -> splineTo(-81.3 + poleOffset.x, -42.2 + poleOffset.y, -42.3)
        1 -> splineTo(-81.1 + poleOffset.x, -42.2 + poleOffset.y, -39.0)
        2 -> splineTo(-80.7 + poleOffset.x, -42.6 + poleOffset.y, -32.8)
        3 -> splineTo(-80.7 + poleOffset.x, -39.5 + poleOffset.y, -32.5)
        4 -> splineTo(-80.1 + poleOffset.x, -39.5 + poleOffset.y, -30.0)
        else -> throw CycleException()
    }

    private fun Anvil.goToIntake(it: Int) = when (it) {
        0 -> splineTo(-161.8, -22.0, 180)
        1 -> splineTo(-161.3, -21.5, 180)
        2 -> splineTo(-161.0, -21.2, 180)
        3 -> splineTo(-160.4, -20.9, 180)
        4 -> splineTo(-160.6, -20.7, 180)
        else -> throw CycleException()
    }.doInReverse()

    private fun Anvil.awaitRegularIntake() = this
        .addTemporalMarker(-170) {
            bot.intake.disable()
        }

        .addTemporalMarker {
            bot.claw.close()
        }

        .addTemporalMarker(275) {
            bot.lift.goToAngledMidButHigher()
        }

        .addTemporalMarker(425) {
            bot.arm.setToForwardsAngledPos()
            bot.wrist.setToForwardsPos()
        }

        .waitTime(300)

    private fun Anvil.awaitFastIntake() = this
        .addTemporalMarker(-275) {
            bot.intake.disable()
        }

        .addTemporalMarker(-75) {
            bot.claw.close()
        }

        .addTemporalMarker(15) {
            bot.arm.setToForwardsAngledPos()
            bot.lift.goToAngledMidButHigher()
        }

        .addTemporalMarker(100) {
            bot.wrist.setToForwardsPos()
        }

        .waitTime(120)

    private fun Anvil.initialDeposit() = this
        .addTemporalMarker(-165) {
            bot.lift.targetHeight -= AutoData.DEPOSIT_DROP_AMOUNT
            bot.arm.setToForwardsPos()
        }
        .addTemporalMarker(50) {
            bot.claw.openForDeposit()
            bot.intake.enable()
        }

    private fun Anvil.deposit(iterations: Int) = this.apply {
        addTemporalMarker(-165) {
            bot.lift.targetHeight -= AutoData.DEPOSIT_DROP_AMOUNT
            bot.arm.setToForwardsPos()
        }

        val durationOffset = if (iterations < 4) -20 else -70

        addTemporalMarker(durationOffset) {
            bot.claw.openForDeposit()
        }
    }

    private fun Anvil.regularIntakePrep(iterations: Int) = this
        .addTemporalMarker(185) {
            when (iterations) {
                0 -> {
                    bot.lift.targetHeight = liftOffsets[iterations]+12
                }
                1 -> {
                    bot.lift.targetHeight = liftOffsets[iterations]+15
                }
                else -> {
                    bot.lift.targetHeight = liftOffsets[iterations]
                }
            }
            bot.wrist.setToBackwardsPos()
            bot.arm.setToBackwardsPosButLikeSliiiightlyHigher()
        }

        .addTemporalMarker(325) {
            bot.claw.openForIntakeNarrow()
            bot.intake.enable()
        }

    private fun Anvil.fastIntakePrep(iterations: Int) = this
        .addTemporalMarker(185) {
            bot.lift.targetHeight = liftOffsets[iterations]

            bot.arm.setToBackwardsPosLastCycle()
            bot.wrist.setToBackwardsPos()

            bot.claw.openForIntakeNarrow()
            bot.intake.enable()
        }

    private fun parkTraj(startPose: Pose2d) =
        Anvil.forgeTrajectory(bot.drive, startPose) {
            resetBot()

            when (signalID) {
                1 -> {
                    lineToLinearHeading(-92.5, -21, 90)
                    lineToLinearHeading(-150, -16, 90)
                }
                2 -> lineToLinearHeading(-92.5, -21, 90)
                3 -> {
                    lineToLinearHeading(-92.5, -21, 90)
                    lineToLinearHeading(-30, -16, 90)
                }
            }

            this
        }

    private fun Anvil.resetBot() = this
        .addTemporalMarker {
            bot.arm.setToRestingPos()
            bot.wrist.setToRestingPos()
            bot.lift.goToZero()
            bot.claw.close()
        }
}
