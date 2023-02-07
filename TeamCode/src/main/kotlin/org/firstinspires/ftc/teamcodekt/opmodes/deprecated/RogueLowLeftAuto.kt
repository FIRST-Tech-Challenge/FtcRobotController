package org.firstinspires.ftc.teamcodekt.opmodes.deprecated

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

@Disabled
@Deprecated("Not that accurate")
@Autonomous
class RogueLowLeftAuto : RogueBaseAuto() {
    override val startPose = GlobalUnits.pos(-91, -163, 90)

    override fun mainTraj(startPose: Pose2d) =
        Anvil.forgeTrajectory(bot.drive, startPose)
            .setVelConstraint(
                SampleMecanumDrive.getVelocityConstraint(
                    42.0,
                    Math.toRadians(260.0),
                    DriveConstants.TRACK_WIDTH
                )
            )
            .addTemporalMarker(0) {
                bot.claw.close()
            }
            .addTemporalMarker(400) {
                bot.lift.goToAngledHigh() // TODO: Change this to mid on real field
                bot.wrist.setToForwardsPos()
            }
            .addTemporalMarker(600) {
                bot.arm.setToForwardsAngledPos()
            }
            .forward(132)
            .turn(-141.5)
            .forward(14)
            .addTemporalMarker(0) {
                bot.arm.setToForwardsPos()
            }
            .addTemporalMarker(100) {
                bot.lift.targetHeight = liftOffsets[0]
            }
            .addTemporalMarker(350) {
                bot.claw.openForDeposit()
            }
            .addTemporalMarker(450) {
                bot.wrist.setToBackwardsPos()
                bot.arm.setToRestingPos()
            }
            .waitTime(350)
            .back(14)
            .turn(51.5)

            .back(70)
            .turn(-27.5)

            .strafeRight(6.6)
            .addTemporalMarker(100) {
                bot.arm.setToBackwardsPos()
                bot.intake.enable()
                bot.claw.openForIntakeNarrow()
            }
            .forward(10.4)
            .waitTime(200)


            .back(10.4)
            .addTemporalMarker(30) {
                bot.claw.close()
                bot.intake.disable()
            }
            .addTemporalMarker(150) {
                bot.arm.setToForwardsPos()
            }
            .addTemporalMarker(350) {
                bot.lift.goToAngledLow()
            }
            .addTemporalMarker(450) {
                bot.wrist.setToForwardsPos()
            }
            .waitTime(400)

            .doTimes(4) {
                goToDeposit(it)
                    .addTemporalMarker(100) {
                        bot.claw.openForDeposit()
                    }
                    .addTemporalMarker(250) {
                        bot.lift.targetHeight = liftOffsets[it + 1]
                        bot.claw.openForIntakeNarrow()
                        bot.intake.enable()
                        bot.wrist.setToBackwardsPos()
                        bot.arm.setToBackwardsPosButLikeSliiiightlyHigher()
                    }
                    .waitTime(500)

                goToIntake(it)
                    .addTemporalMarker(30) {
                        bot.claw.close()
                        bot.intake.disable()
                    }
                    .addTemporalMarker(140) {
                        bot.arm.setToForwardsPos()
                    }
                    .addTemporalMarker(350) {
                        bot.lift.goToAngledLow()
                    }
                    .addTemporalMarker(415) {
                        bot.wrist.setToForwardsPos()
                    }
                    .waitTime(400)
            }
            .goToDeposit(4)
            .addTemporalMarker(100) {
                bot.claw.openForDeposit()
            }
            .addTemporalMarker(250) {
                bot.lift.goToZero()
                bot.arm.setToRestingPos()
                bot.wrist.setToBackwardsPos()
            }
            .waitTime(500)

            .resetBot()


            .thenRun(::parkTraj)


    private fun Anvil.goToDeposit(it: Int) = when (it) {
        /*
            The offset values are from sin(32) and cos(32) degrees.
            Used to spline in a straight line. This is advantageous to maintain localization better.
        */
        0 -> forward(AutoData.LOW_DEPOSIT_1)
        1 -> forward(AutoData.LOW_DEPOSIT_2)
        2 -> forward(AutoData.LOW_DEPOSIT_3)
        3 -> forward(AutoData.LOW_DEPOSIT_4)
        4 -> forward(AutoData.LOW_DEPOSIT_5)

        else -> throw CycleException()
    }

    private fun Anvil.goToIntake(it: Int) = when (it) {

        0 -> back(AutoData.LOW_INTAKE_1)
        1 -> back(AutoData.LOW_INTAKE_2)
        2 -> back(AutoData.LOW_INTAKE_3)
        3 -> back(AutoData.LOW_INTAKE_4)
        4 -> back(AutoData.LOW_INTAKE_5)
        else -> throw CycleException()
    }


    private fun Anvil.resetBot() = this
        .addTemporalMarker {
            bot.arm.setToRestingPos()
            bot.wrist.setToRestingPos()
            bot.lift.goToZero()
            bot.claw.close()
        }

    private fun parkTraj(startPose: Pose2d) =
        Anvil.forgeTrajectory(bot.drive, startPose) {
            resetBot()

            when (signalID) {
                1 -> lineToLinearHeading(-160, -33, -90)
                2 -> lineToLinearHeading(-95.5, -24, -90)
                3 -> lineToLinearHeading(-30, -34, -90)
            }

            this
        }
}
