package org.firstinspires.ftc.teamcodekt.opmodes.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import ftc.rogue.blacksmith.Anvil
import ftc.rogue.blacksmith.Scheduler
import ftc.rogue.blacksmith.units.DistanceUnit
import ftc.rogue.blacksmith.util.kt.toIn
import ftc.rogue.blacksmith.util.kt.toRad

class AnvilRightTestAuto : RogueBaseAuto() {
    override fun goo() = with(bot) {
        Anvil.setUnits(distanceUnit = DistanceUnit.CM)

        val startPose = Pose2d(91.toIn(), (-159).toIn(), 90.toRad())
        val startTraj = preload(startPose)

        Anvil.startAutoWith(startTraj)

        waitForStart()

        Anvil.start()

        Scheduler.launch(this@AnvilRightTestAuto) {
            bot.updateComponents(mTelemetry)
            drive.update()
            mTelemetry.update()
        }
    }

    private fun preload(startPose: Pose2d): Anvil =
        Anvil.formTrajectory(bot.drive, startPose)
            .preform(key = 0, ::intakeCycle)

            .splineToLinearHeading(87.25, -22.0, 45.0, 10.0)

            .thenRunPreformed(key = 0)

    private fun intakeCycle(startPose: Pose2d): Anvil =
        Anvil.formTrajectory(bot.drive, startPose)
            .preform(key = 0, ::depositCycle)

            .inReverse {
                splineTo(87.25, -50.0, 270.0)
            }

            .thenRunPreformed(key = 0)

    private fun depositCycle(startPose: Pose2d): Anvil =
        Anvil.formTrajectory(bot.drive, startPose)
            .splineTo(87.25, -22.0, 45.0)
}
