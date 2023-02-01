package org.firstinspires.ftc.teamcodekt.opmodes.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import ftc.rogue.blacksmith.Anvil
import ftc.rogue.blacksmith.units.GlobalUnits
import org.firstinspires.ftc.teamcode.AutoData
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcodekt.components.LIFT_HIGH
import org.firstinspires.ftc.teamcodekt.components.meta.createAutoBotComponents
import org.firstinspires.ftc.teamcodekt.opmodes.auto.RogueBaseAuto
import org.firstinspires.ftc.teamcodekt.util.CycleException

@Autonomous
class RogueStraightMidLeftAuto : RogueBaseAuto() {
    override val startPose = GlobalUnits.pos(-91, -163, 90)

    override fun mainTraj(startPose: Pose2d) =
            Anvil.formTrajectory(bot.drive, startPose)
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(36.0, Math.toRadians(260.0), DriveConstants.TRACK_WIDTH))
                    .preform(0, ::parkTraj)
                    .addTemporalMarker(0) {
                        bot.claw.close()
                    }
                    .addTemporalMarker(600) {
                        bot.lift.goToAngledHigh() // TODO: Change this to mid on real field
                        bot.arm.setToForwardsAngledPos()
                        bot.wrist.setToForwardsPos()
                    }
                    .forward(132)
                    .turn(-141.5)
                    .forward(14)
                    .addTemporalMarker(0) {
                        bot.arm.setToForwardsPos()
                        bot.lift.targetHeight = LIFT_HIGH - 500
                        bot.lift.regenMotionProfile()
                    }
                    .addTemporalMarker(100) {
                        bot.lift.targetHeight = liftOffsets[0]
                        bot.lift.regenMotionProfile()
                        bot.arm.setToBackwardsPosButLikeSliiiightlyHigher()
                    }
                    .addTemporalMarker(350) {
                        bot.claw.openForDeposit()
                    }
                    .addTemporalMarker(450) {
                        bot.wrist.setToBackwardsPos()
                    }
                    .waitTime(350)
                    .back(14)
                    .turn(51.5)

                    .back(70)

                    .addTemporalMarker(100) {
                        bot.arm.setToBackwardsPosButLikeSliiiightlyHigher()
                        bot.claw.openForIntakeWide()
                    }
                    .forward(10.4)
                    .waitTime(200)

                    .back(10.4)
                    .addTemporalMarker(30) {
                        bot.claw.close()
                    }
                    .addTemporalMarker(200) {
                        bot.arm.setToForwardsAngledPos()
                    }
                    .addTemporalMarker(350) {
                        bot.lift.goToAngledHigh()
                    }
                    .addTemporalMarker(450) {
                        bot.wrist.setToForwardsPos()
                    }
                    .waitTime(400)

                    .doTimes(4) {
                        goToDeposit(it)
                                .addTemporalMarker(0) {
                                    bot.arm.setToForwardsPos()
                                    bot.lift.targetHeight = LIFT_HIGH - 1000
                                    bot.lift.regenMotionProfile()
                                }

                                .addTemporalMarker(100) {
                                    bot.claw.openForDeposit()
                                }
                                .addTemporalMarker(250) {
                                    bot.lift.targetHeight = liftOffsets[it + 1]
                                    bot.lift.regenMotionProfile()
                                    bot.claw.openForIntakeWide()
                                    bot.wrist.setToBackwardsPos()
                                    bot.arm.setToBackwardsPosButLikeSliiiightlyHigher()
                                }
                                .waitTime(500)

                        goToIntake(it)
                                .addTemporalMarker(30) {
                                    bot.claw.close()
                                }
                                .addTemporalMarker(200) {
                                    bot.arm.setToForwardsAngledPos()
                                }
                                .addTemporalMarker(375) {
                                    bot.lift.goToAngledHigh()
                                }
                                .addTemporalMarker(415) {
                                    bot.wrist.setToForwardsPos()
                                }
                                .waitTime(400)
                    }
                    .goToDeposit(4)
                    .addTemporalMarker(0) {
                        bot.arm.setToForwardsPos()
                        bot.lift.targetHeight = LIFT_HIGH - 1000
                        bot.lift.regenMotionProfile()
                    }
                    .addTemporalMarker(225) {
                        bot.claw.openForDeposit()
                    }
                    .addTemporalMarker(450) {
                        bot.arm.setToBackwardsPosButLikeSliiiightlyHigher()
                        bot.wrist.setToBackwardsPos()
                    }
                    .waitTime(500)

                    .resetBot()


                    .thenRunPreformed(0)


    private fun Anvil.goToDeposit(it: Int) = when (it) {
        /*
            The offset values are from sin(32) and cos(32) degrees.
            Used to spline in a straight line. This is advantageous to maintain localization better.
        */
        0 -> splineTo(-85.5, -41, -37)
        1 -> splineTo(-85.5, -41, -35)
        2 -> splineTo(-85.5, -41, -33)
        3 -> splineTo(-85.5, -41, -31)
        4 -> splineTo(-85.5, -41, -30)

        else -> throw CycleException()
    }

    private fun Anvil.goToIntake(it: Int) = when (it) {
        0 -> splineTo(-164, -28, 180)
        1 -> splineTo(-164, -28, 180)
        2 -> splineTo(-164, -28, 180)
        3 -> splineTo(-164, -28, 180)
        4 -> splineTo(-164, -28, 180)
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
                    1 -> lineToLinearHeading(-160, -33, -90)
                    2 -> lineToLinearHeading(-95.5, -24, -90)
                    3 -> lineToLinearHeading(-30, -34, -90)
                }

                this
            }
}
