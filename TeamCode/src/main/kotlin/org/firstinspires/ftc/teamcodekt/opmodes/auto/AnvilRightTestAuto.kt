package org.firstinspires.ftc.teamcodekt.opmodes.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import ftc.rogue.blacksmith.Anvil
import ftc.rogue.blacksmith.Scheduler
import ftc.rogue.blacksmith.util.bsmPose2d
import ftc.rogue.blacksmith.util.kt.toIn
import ftc.rogue.blacksmith.util.kt.toRad

@Autonomous
class AnvilRightTestAuto : RogueBaseAuto() {
    override fun executeOrder66() {
        val startPose = bsmPose2d(91, -159, 90)
        val startTraj = mainTraj(startPose)

        Anvil.startAutoWith(startTraj).onSchedulerLaunch()

        Scheduler.launchOnStart(opmode = this) {
            bot.updateComponents(mTelemetry)
            bot.drive.update()
            mTelemetry.update()
        }
    }

    private fun mainTraj(startPose: Pose2d): Anvil =
        Anvil.formTrajectory(bot.drive, startPose)
            .splineToSplineHeading(75.25, -17.0, 135.0, 117.5) // Preload

            .doTimes(5) {
                inReverse {
                    splineTo(153.5, -29.5, 0.0) // Intake
                }
                splineToSplineHeading(77.25, -17.0, 135.0, 135.0) // Deposit
            }
}
