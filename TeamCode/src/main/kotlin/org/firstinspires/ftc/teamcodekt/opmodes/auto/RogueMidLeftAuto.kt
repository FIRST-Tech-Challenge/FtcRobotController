package org.firstinspires.ftc.teamcodekt.opmodes.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import ftc.rogue.blacksmith.Anvil
import ftc.rogue.blacksmith.units.GlobalUnits
import org.firstinspires.ftc.teamcode.AutoData
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcodekt.util.CycleException

@Autonomous
class RogueMidLeftAuto : RogueBaseAuto() {
    override val startPose = GlobalUnits.pos(-91, -163, 90)

    override fun mainTraj(startPose: Pose2d) =
            Anvil.formTrajectory(bot.drive, startPose)
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40.0, Math.toRadians(250.0), DriveConstants.TRACK_WIDTH))
                    .preform(0, ::parkTraj)
                    .addTemporalMarker {
                        bot.lift.goToAngledHigh()
                        bot.claw.close()
                        bot.arm.setToForwardsAngledPos()
                        bot.wrist.setToForwardsPos()
                    }

                    .forward(132)
                    .turn(-142.5)
                    .goToDeposit(-1)
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

                    .resetBot()


                    .thenRunPreformed(0)


    private fun Anvil.goToDeposit(it: Int) = when (it) {
        -1 -> lineToLinearHeading(-83.2, -43.9, -39.5)
        0 -> splineTo(-81, -43, -38)
        1 -> splineTo(-81, -43, -36)
        2 -> splineTo(-82, -43.1, -31)
        3 -> splineTo(-81, -42, -30)
        4 -> splineTo(-82, -41.5, -28)

        else -> throw CycleException()
    }

    private fun Anvil.goToIntake(it: Int) = when (it) {
        0 -> splineTo(-166.3, -25, 180)
        1 -> splineTo(-165.9, -24.5, 180)
        2 -> splineTo(-164.2, -24.8, 180)
        3 -> splineTo(-165.5, -23.9, 180)
        4 -> splineTo(-164.3, -23.7, 180)
        else -> throw CycleException()
    }.doInReverse()

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

    private fun Anvil.initialDeposit() = this
            .addTemporalMarker(-165) {
                bot.lift.targetHeight -= AutoData.DEPOSIT_DROP_AMOUNT
                bot.arm.setToForwardsPos()
            }
            .addTemporalMarker(-100) {
                bot.claw.openForDeposit()
            }

    private fun Anvil.deposit(iterations: Int) = this
        .addTemporalMarker(-165) {
            bot.lift.targetHeight -= AutoData.DEPOSIT_DROP_AMOUNT
            bot.arm.setToForwardsPos()
        }
        .addTemporalMarker(-150) {
            if(iterations == 4)
                bot.claw.openForDeposit()

        }
        .addTemporalMarker(-100) {
            if(iterations < 4)
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
                        lineToLinearHeading(-160, -20, -90)
                        lineToLinearHeading(-95.5, -20, -90)
                    }
                    2 -> lineToLinearHeading(-95.5, -20, -90)
                    3 -> lineToLinearHeading(-30, -20, -90)
                }

                this
            }
}
