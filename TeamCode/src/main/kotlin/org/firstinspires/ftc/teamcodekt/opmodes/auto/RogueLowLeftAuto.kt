package org.firstinspires.ftc.teamcodekt.opmodes.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import ftc.rogue.blacksmith.Anvil
import ftc.rogue.blacksmith.units.GlobalUnits
import ftc.rogue.blacksmith.util.toRad
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
            .setVelConstraint(36, 260.toRad(), DriveConstants.TRACK_WIDTH)
            .preform(0, ::parkTraj)

            .addTemporalMarker(600) {
                bot.lift.goToHigh() // TODO: Change this to mid on real field
                bot.arm.setToForwardsPos()
                bot.wrist.setToForwardsPos()
            }

            .forward(132)
            .turn(-147)
            .forward(3)

            .addTemporalMarker(150) {
                bot.lift.targetHeight = LIFT_HIGH - 1000
            }
            .addTemporalMarker(250) {
                bot.claw.openForDeposit()
            }
            .waitTime(300)
            .addTemporalMarker(50) {
                bot.lift.targetHeight = liftOffsets[0]
                bot.wrist.setToBackwardsPos()
            }

            .back(3)
            .turn(57)

            .back(70)
            .turn(-28)

            .strafeRight(5.5)
            .forward(10)

            .addTemporalMarker(50) {
                bot.arm.setToBackwardsPos()
                bot.claw.openForIntakeWide()
            }
            .waitTime(300)

            .back(12)

            .addTemporalMarker(40) {
                bot.claw.close()
            }
            .addTemporalMarker(100) {
                bot.lift.goToAngledLow()
            }
            .addTemporalMarker(200) {
                bot.arm.setToForwardsPos()
                bot.wrist.setToForwardsPos()
            }
            .waitTime(200)

            .doTimes(4) {
                goToDeposit(it)
                addTemporalMarker(150) {
                    bot.claw.openForDeposit()
                }
                addTemporalMarker(250) {
                    bot.lift.targetHeight = liftOffsets[it]
                    bot.claw.openForIntakeWide()
                    bot.wrist.setToBackwardsPos()
                    bot.arm.setToBackwardsPos()
                }
                waitTime(500)

                goToIntake(it)
                addTemporalMarker(100) {
                    bot.claw.close()
                }
                addTemporalMarker(200) {
                    bot.lift.goToAngledLow()
                }
                addTemporalMarker(300) {
                    bot.arm.setToForwardsPos()
                    bot.wrist.setToForwardsPos()
                }
                waitTime(400)
            }
            .goToDeposit(4)

            .resetBot()

    //  The offset values are from sin(32) and cos(32) degrees.
    //  Used to spline in a straight line. This is advantageous to maintain localization better.
    private fun Anvil.goToDeposit(it: Int) = when (it) {
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
                    splineTo(-160, -33, 180)
                }
                2 -> inReverse {
                    splineTo(-95.5, -24, 180)
                }
                3 -> {
                    splineToLinearHeading(-30, -34, -90, -38.375)
                }
            }

            this
        }
}
