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
class RogueMidRightAuto : RogueBaseAuto() {
    override val startPose = GlobalUnits.pos(91, -163, 90)

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
                    .turn(142.5)
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
        -1 -> lineToLinearHeading(83.2+poleOffset.x, -43.9+poleOffset.y, 180+39.5)
        0 -> splineTo(81+poleOffset.x, -45+poleOffset.y, 180+38)
        1 -> splineTo(81+poleOffset.x, -45+poleOffset.y, 180+36)
        2 -> splineTo(82+poleOffset.x, -45.1+poleOffset.y, 180+32.5)
        3 -> splineTo(82+poleOffset.x, -45.5+poleOffset.y, 180+32.5)
        4 -> splineTo(82+poleOffset.x, -47.5+poleOffset.y, 180+32)

        else -> throw CycleException()
    }

    private fun Anvil.goToIntake(it: Int) = when (it) {
        0 -> splineTo(162.8, -27.3, 0)
        1 -> splineTo(162.0, -27.3, 0)
        2 -> splineTo(161.7, -27.3, 0)
        3 -> splineTo(161.5, -27.3, 0)
        4 -> splineTo(161.9, -26.7, 0)
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
            bot.lift.goToAngledHigh()
        }

        .addTemporalMarker(425) {
            bot.arm.setToForwardsAngledPos()
            bot.wrist.setToForwardsPos()
        }

        .waitTime(300)

    private fun Anvil.awaitFastIntake() = this
        .addTemporalMarker(-245){
            bot.intake.disable()
        }
        .addTemporalMarker(-75) {
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
                bot.intake.enable()
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
                when(iterations){
                    1 -> bot.lift.targetHeight = liftOffsets[iterations]+9
                    2 -> bot.lift.targetHeight = liftOffsets[iterations]+19
                    3 -> bot.lift.targetHeight = liftOffsets[iterations]-11
//                    4 -> bot.lift.targetHeight = liftOffsets[iterations]-14
                    else -> bot.lift.targetHeight = liftOffsets[iterations]

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
                        lineToLinearHeading(95.5, -20, -90)
                        lineToLinearHeading(30, -20, -90)
                    }
                    2 -> lineToLinearHeading(95.5, -20, -90)
                    3 -> lineToLinearHeading(160, -20, -90)

                }

                this
            }
}
