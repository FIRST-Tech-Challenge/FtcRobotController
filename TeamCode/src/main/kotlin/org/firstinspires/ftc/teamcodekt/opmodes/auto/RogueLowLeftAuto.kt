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
import org.firstinspires.ftc.teamcodekt.opmodes.auto.RogueBaseAuto
import org.firstinspires.ftc.teamcodekt.util.CycleException

@Autonomous
class RogueLowLeftAuto : RogueBaseAuto() {
    override val startPose = GlobalUnits.pos(-91, -163, 90)

    override fun mainTraj(startPose: Pose2d) =
            Anvil.formTrajectory(bot.drive, startPose)
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(36.0, Math.toRadians(260.0), DriveConstants.TRACK_WIDTH))
            .preform(0, ::parkTraj)
                    .addTemporalMarker(600) {
                        bot.lift.goToHigh() // TODO: Change this to mid on real field
                        bot.arm.setToForwardsPos()
                        bot.wrist.setToForwardsPos()
                    }
                    .forward(132)
                    .turn(-142)
                    .forward(2)
                    .addTemporalMarker(100) {
                        bot.lift.targetHeight = liftOffsets[0]
                        bot.lift.regenMotionProfile()
                    }
                    .addTemporalMarker(200) {
                        bot.claw.openForDeposit()
                    }
                    .addTemporalMarker(400) {
                        bot.wrist.setToBackwardsPos()
                    }
                    .waitTime(300)
                    .back(4.5)
                    .turn(52)

                    .back(70)
                    .turn(-28)

                    .strafeRight(5.5)
                    .forward(10)
                    .addTemporalMarker(50) {
                        bot.arm.setToBackwardsPos()
                        bot.claw.openForIntakeWide()
                    }
                    .waitTime(300)


                    .back(9.75)
                    .addTemporalMarker(100) {
                        bot.claw.close()
                    }
                    .addTemporalMarker(200) {
                        bot.lift.goToAngledLow()
                    }
                    .addTemporalMarker(300) {
                        bot.arm.setToForwardsPos()
                    }
                    .addTemporalMarker(415) {
                        bot.wrist.setToForwardsPos()
                    }
                    .waitTime(400)

                    .doTimes(4) {
                        goToDeposit(it)
                        .addTemporalMarker(100) {
                            bot.claw.openForDeposit()
                        }
                        .addTemporalMarker(250) {
                            bot.lift.targetHeight = liftOffsets[it]
                            bot.lift.regenMotionProfile()
                            bot.claw.openForIntakeWide()
                            bot.wrist.setToBackwardsPos()
                            bot.arm.setToBackwardsPos()
                        }
                        .waitTime(500)

                        goToIntake(it)
                        .addTemporalMarker(100) {
                            bot.claw.close()
                        }
                        .addTemporalMarker(200) {
                            bot.lift.goToAngledLow()
                        }
                        .addTemporalMarker(300) {
                            bot.arm.setToForwardsPos()
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


            .thenRunPreformed(0)


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

        else -> throw CycleException()
    }
    private fun Anvil.goToIntake(it: Int) = when (it) {

        0 -> back(21.5)
        1 -> back(21.5)
        2 -> back(21.5)
        3 -> back(21.5)
        4 -> back(21.5)
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
                        lineToLinearHeading(-160, -33, 180)
                    }
                    2 -> inReverse {
                        lineToLinearHeading(-95.5, -24, 180)
                    }
                    3 -> {
                        lineToLinearHeading(-30, -34, 180)
                    }
                }

                this
            }
}
