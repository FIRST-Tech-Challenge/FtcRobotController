package org.firstinspires.ftc.teamcodekt.opmodes.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import ftc.rogue.blacksmith.Scheduler
import ftc.rogue.blacksmith.util.toIn
import ftc.rogue.blacksmith.util.toRad

@Autonomous
class NormalRightTestAuto : RogueBaseAuto() {
    override fun goo() = with(bot) {
        val startPose = Pose2d(91.toIn(), (-159).toIn(), 90.toRad())
        val mainTraj = mainTraj(startPose).build()

        waitForStart()

        drive.poseEstimate = startPose
        drive.followTrajectorySequenceAsync(mainTraj)

        Scheduler.launch(this@NormalRightTestAuto) {
            bot.updateComponents(mTelemetry)
            drive.update()
            mTelemetry.update()
        }
    }

    private fun mainTraj(startPose: Pose2d) = bot.drive.trajectorySequenceBuilder(startPose)
        .splineToLinearHeading( Pose2d(87.25.toIn(), (-22).toIn(), 45.toRad()), 10.toRad())

        .setReversed(true)
        .splineTo( Vector2d(87.25.toIn(), (-50).toIn()), 270.toRad() )
        .setReversed(false)

        .splineTo( Vector2d(87.25.toIn(), (-22).toIn()), 45.toRad() )
}
