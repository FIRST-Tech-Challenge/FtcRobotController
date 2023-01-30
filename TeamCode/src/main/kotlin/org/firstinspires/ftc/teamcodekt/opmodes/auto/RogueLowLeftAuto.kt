package org.firstinspires.ftc.teamcodekt.opmodes.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import ftc.rogue.blacksmith.Anvil
import ftc.rogue.blacksmith.units.GlobalUnits
import org.firstinspires.ftc.teamcode.AutoData
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive

@Autonomous
class RogueLowLeftAuto : RogueBaseAuto() {
    override val startPose = GlobalUnits.pos(-91, -163, 90)

    override fun mainTraj(startPose: Pose2d) =
            Anvil.formTrajectory(bot.drive, startPose)
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(36.0, Math.toRadians(260.0), DriveConstants.TRACK_WIDTH))
            .preform(0, ::parkTraj)
                    .awaitInitialGoToDeposit()
                    .turn(46)
                    .addTemporalMarker(0) {
                        bot.lift.clippedHeight = liftOffsets[0]-8
                        bot.wrist.setToBackwardsPos()
                    }
                    .back(70)
                    .strafeRight(4)
                    .turn(-29)
                    .forward(10)
                    .regularIntakePrep(0)


                    .back(16.5)
                    .awaitRegularIntake()

                    /*
                        Loop is arranged a bit strangely - because intaking initially is a bit different, the last
                        deposit is managed automatically
                     */
                    .doTimes(4) {
                        goToDeposit(it+1)
                        deposit()
                        regularIntakePrep(it+1)
                        goToIntake(it+1)
                        awaitRegularIntake()
                        waitTime(200)
                    }
                    .goToDeposit(4)
                    .deposit()

                    .resetBot()


            .thenRunPreformed(0)

    private fun Anvil.initialDepositPrep() = this
            .addTemporalMarker {
                bot.lift.goToAngledMid()
                bot.claw.close()
                bot.arm.setToForwardsAngledPos()
                bot.wrist.setToForwardsPos()
            }

    private fun Anvil.awaitInitialGoToDeposit() = this
            .initialDepositPrep()
            .forward(132)
            .turn(-136)
            .forward(12.5)
            .waitTime(150)
            .deposit()
            .back(12.5)


    private fun Anvil.goToDeposit(it: Int) = when (it) {
        /*
            The offset values are from sin(32) and cos(32) degrees.
            Used to spline in a straight line. This is advantageous to maintain localization better.
        */
        0 -> forward(21.5)
        1 -> forward(21.5)
        2 -> forward(21.5)
        3 -> forward(21.5)
        4 -> forward(21.5)

        else -> noop
    }
    private fun Anvil.goToIntake(it: Int) = when (it) {

        0 -> back(21.5)
        1 -> back(21.5)
        2 -> back(21.5)
        3 -> back(21.5)
        4 -> back(21.5)
        else -> noop
    }.doInReverse()

    private fun Anvil.deposit() = this
            .addTemporalMarker(-165) {
                bot.lift.clippedHeight -= AutoData.DEPOSIT_DROP_AMOUNT
            }

            .addTemporalMarker(-100) {
                bot.claw.openForDeposit()
            }
            .waitTime(100)


    private fun Anvil.regularIntakePrep(iterations: Int) = this
            .addTemporalMarker(0) {
                bot.arm.setToBackwardsPosButLikeSliiiightlyHigher()
                bot.claw.openForIntakeWide()
            }

            .addTemporalMarker(105) {
                bot.lift.clippedHeight = liftOffsets[iterations]-15
                bot.wrist.setToBackwardsPos()
            }

            .addTemporalMarker(60) {
                bot.intake.disable()
                bot.claw.openForIntakeWide()
            }

            .waitTime(200)

    private fun Anvil.awaitRegularIntake() = this
            .addTemporalMarker {
                bot.intake.disable()
                bot.claw.close()
            }

            .addTemporalMarker(245) {
                bot.lift.goToAngledLow()
            }

            .addTemporalMarker(425) {
                bot.arm.setToForwardsAngledPos()
                bot.wrist.setToForwardsPos()
            }

            .waitTime(300)

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
                        splineTo(-95.5, -24, 180)
                    }
                    3 -> {
                        splineToLinearHeading(-30, -34, 90, -38.375)
                    }
                }

                this
            }
}
