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

                    .forward(132)
                    .turn(-141.5)
                    .goToDeposit(-1)


                    .doTimes(5) {
                        goToIntake(it)
                        waitTime(400)
                        goToDeposit(it)
                        waitTime(500)
                    }

                    .resetBot()


                    .thenRunPreformed(0)


    private fun Anvil.goToDeposit(it: Int) = when (it) {
        /*
            The offset values are from sin(32) and cos(32) degrees.
            Used to spline in a straight line. This is advantageous to maintain localization better.
        */
        -1 -> lineToLinearHeading(-85.5, -40.5, -37)
        0 -> splineTo(-86, -40.5, -37)
        1 -> splineTo(-86, -40.5, -35)
        2 -> splineTo(-86, -40.5, -30)
        3 -> splineTo(-86, -39.7, -29)
        4 -> splineTo(-86, -39.2, -27)

        else -> throw CycleException()
    }

    private fun Anvil.goToIntake(it: Int) = when (it) {
        0 -> splineTo(-165, -26.75, 180)
        1 -> splineTo(-165, -25.3, 180)
        2 -> splineTo(-164, -25, 180)
        3 -> splineTo(-164.1, -24.1, 180)
        4 -> splineTo(-164.1, -23.9, 180)
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
