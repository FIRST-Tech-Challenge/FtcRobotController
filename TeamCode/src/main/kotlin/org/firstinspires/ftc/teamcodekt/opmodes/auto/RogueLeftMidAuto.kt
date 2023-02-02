package org.firstinspires.ftc.teamcodekt.opmodes.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import ftc.rogue.blacksmith.Anvil
import ftc.rogue.blacksmith.units.GlobalUnits
import org.firstinspires.ftc.teamcode.AutoData
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcodekt.util.CycleException

@Autonomous
class RogueLeftMidAuto : RogueBaseAuto() {
    override val startPose = GlobalUnits.pos(-84, -169, 90)

    override fun mainTraj(startPose: Pose2d) =
            Anvil.formTrajectory(bot.drive, startPose)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(43.0, Math.toRadians(260.0), DriveConstants.TRACK_WIDTH))
                .forward(15)
                .turn(180)
                .goToDeposit(-1)
                .deposit()

                .splineTo(-73.2, -47.8, -44)

                .doTimes(5) {
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
                    deposit()
                }

    private fun Anvil.goToDeposit(it: Int) = when (it) {
        -1 -> inReverse {
            splineTo(-84.5, -36.5, 90)
            turn(44)
        }
        0 -> splineTo(-73.2, -47.8, -44)
        1 -> splineTo(-73.2, -47.8, -44)
        2 -> splineTo(-73.2, -47.8, -44)
        3 -> splineTo(-73.2, -47.8, -44)
        4 -> splineTo(-73.2, -47.8, -44)

        else -> throw CycleException()
    }

    private fun Anvil.goToIntake(it: Int) = when (it) {
        0 -> splineTo(-156, -30, 180)
        1 -> splineTo(-156, -30, 180)
        2 -> splineTo(-156, -30, 180)
        3 -> splineTo(-156, -30, 180)
        4 -> splineTo(-156, -30, 180)
        else -> throw CycleException()
    }.doInReverse()


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
                    splineTo(-160, -33, 180)
                }
                2 -> inReverse {
                    splineTo(-95.5, -30, 180)
                }
                3 -> {
                    splineToLinearHeading(-30, -34, 90, -38.375)
                }
            }

            this
        }

    private fun Anvil.awaitRegularIntake() = this
            .addTemporalMarker {
                bot.intake.disable()
                bot.claw.close()
            }

            .addTemporalMarker(275) {
                bot.lift.goToAngledHigh()
            }

            .addTemporalMarker(425) {
                bot.arm.setToForwardsAngledPos()
                bot.wrist.setToForwardsPos()
            }

            .waitTime(300)

    private fun Anvil.awaitFastIntake() = this
            .addTemporalMarker(-75) {
                bot.intake.disable()
                bot.claw.close()
            }

            .addTemporalMarker(15) {
                bot.arm.setToForwardsAngledPos()
                bot.lift.goToAngledHigh()
            }

            .addTemporalMarker(100) {
                bot.wrist.setToForwardsPos()
            }

            .waitTime(120)

    private fun Anvil.deposit() = this
            .addTemporalMarker(-165) {
                bot.lift.targetHeight -= AutoData.DEPOSIT_DROP_AMOUNT
                bot.arm.setToForwardsPos()
            }
            .addTemporalMarker(-100) {
                bot.claw.openForDeposit()
            }


    private fun Anvil.regularIntakePrep(iterations: Int) = this
            .addTemporalMarker(185) {
                bot.lift.targetHeight = liftOffsets[iterations]
                bot.wrist.setToBackwardsPos()
                bot.arm.setToBackwardsPosButLikeSliiiightlyHigher()
            }

            .addTemporalMarker(325) {
                bot.claw.openForIntakeWide()
            }

    private fun Anvil.fastIntakePrep(iterations: Int) = this
            .addTemporalMarker(185) {
                bot.lift.targetHeight  = liftOffsets[iterations]

                bot.arm.setToBackwardsPos()
                bot.wrist.setToBackwardsPos()

                bot.claw.openForIntakeNarrow()
                bot.intake.enable()
            }
}
