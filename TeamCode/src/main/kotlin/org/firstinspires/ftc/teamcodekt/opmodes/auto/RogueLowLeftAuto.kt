package org.firstinspires.ftc.teamcodekt.opmodes.auto

import com.acmerobotics.roadrunner.drive.Drive
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
//            .preform(0, ::parkTraj)
                .awaitInitialGoToDeposit()
                .turn(45)
                .addTemporalMarker(0) {
                    bot.lift.height = liftOffsets[0]
                    bot.wrist.setToBackwardsPos()
                }
                .back(70)
                .strafeRight(4)
                .addTemporalMarker(0) {
                    bot.arm.setToBackwardsPosButLikeSliiiightlyHigher()
                }
                .turn(-30)


                .back(5)
                .awaitRegularIntake()

                /*
                    Loop is arranged a bit strangely - because intaking initially is a bit different, the last
                    deposit is managed automatically
                 */
                .doTimes(RogueBaseAuto.NUM_CYCLES-1) {
                    goToDeposit(it+1)
                    deposit()
                    waitTime(200)
                    regularIntakePrep(it+1)
                    goToIntake(it+1)
                    awaitRegularIntake()
                    waitTime(200)
                }

                .resetBot()
                .waitTime(1000)
//                // Final deposit - must be tweaked here
//                .splineTo(-151+11.8727, -42-7.4189, -32)

//            .thenRunPreformed(0)

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
        .turn(-135)
        .forward(13.5)
        .waitTime(150)
        .deposit()
        .back(13.5)


    private fun Anvil.goToDeposit(it: Int) = when (it) {
        /*
            The offset values are from sin(32) and cos(32) degrees.
            Used to spline in a straight line. This is advantageous to maintain localization better.
        */
        0 -> forward(21.5)
        1 -> forward(21.5)
        2 -> forward(21.5)
        3 -> forward(21.5)

        else -> noop
    }
    private fun Anvil.goToIntake(it: Int) = when (it) {

        0 -> back(21.5)
        1 -> back(21.5)
        2 -> back(21.5)
        3 -> back(21.5)
        else -> noop
    }.doInReverse()

    private fun Anvil.deposit() = this
            .addTemporalMarker(-165) {
                bot.lift.height -= AutoData.DEPOSIT_DROP_AMOUNT
            }

            .addTemporalMarker(-100) {
                bot.claw.openForDeposit()
            }
            .waitTime(100)


    private fun Anvil.regularIntakePrep(iterations: Int) = this
        .addTemporalMarker(185) {
            bot.lift.height = liftOffsets[iterations]
            bot.wrist.setToBackwardsPos()
            bot.arm.setToBackwardsPosButLikeSliiiightlyHigher()
        }

        .addTemporalMarker(325) {
            bot.claw.openForIntakeKindaWide()
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
            bot.lift.goToAngledLow()
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
