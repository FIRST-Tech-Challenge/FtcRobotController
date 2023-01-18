package org.firstinspires.ftc.teamcodekt.opmodes.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import ftc.rogue.blacksmith.Anvil
import ftc.rogue.blacksmith.Scheduler
import ftc.rogue.blacksmith.units.GlobalUnits
import kotlin.properties.Delegates

@Autonomous
class RogueLeftParkAuto : RogueBaseAuto() {
    private var signalID by Delegates.notNull<Int>()

    override fun executeOrder66() {
        val startPose = GlobalUnits.pos(91, -159, 90)
        val startTraj = parkTraj(startPose)

        Anvil.startAutoWith(startTraj).onSchedulerLaunch()

        bot.camera.update()
        signalID = bot.camera.waitForStartWithVision(this) ?: 2

        Scheduler.launch(this, ::updateComponents)
    }

    private fun parkTraj(startPose: Pose2d) =
        Anvil.formTrajectory(bot.drive, startPose) {
            forward(74.0)

            when (signalID) {
                1 -> strafeLeft(60.0)
                3 -> strafeRight(60.0)
            }

            this
        }
}
