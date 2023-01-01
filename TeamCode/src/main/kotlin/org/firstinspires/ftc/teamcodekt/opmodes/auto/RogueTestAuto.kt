package org.firstinspires.ftc.teamcodekt.opmodes.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import ftc.rogue.blacksmith.Anvil
import ftc.rogue.blacksmith.Scheduler
import ftc.rogue.blacksmith.units.DistanceUnit

class RogueTestAuto : RogueBaseAuto() {
    override fun goo() = with(bot) {
        Anvil.setUnits(distanceUnit = DistanceUnit.CM)

        val startPose = Pose2d()
        val startTraj = traj2(startPose)

        Anvil.startAutoWith(startTraj)

        waitForStart()

        Anvil.start()

        Scheduler.launch(this@RogueTestAuto) {
            bot.updateComponents(mTelemetry)
            drive.update()
            mTelemetry.update()
        }
    }

    private fun traj1(startPose: Pose2d): Anvil =
        Anvil.formTrajectory(bot.drive, startPose)
            .preform(key = 0, ::traj2)

            .forward(10.0)

            .thenRunPreformed(key = 0)

    private fun traj2(startPose: Pose2d): Anvil =
        Anvil.formTrajectory(bot.drive, startPose)
            .preform(key = 0, ::traj3)

            .forward(10.0)
            .waitTime(3)

            .inReverse {
                forward(10.0)
            }

            .thenRunPreformed(key = 0)

    private fun traj3(startPose: Pose2d): Anvil =
        Anvil.formTrajectory(bot.drive, startPose)
            .preform(key = 0, ::traj3)

            .doTimes(4) {
                turn(90.0)
            }

            .thenRunPreformed(key = 0)
}
