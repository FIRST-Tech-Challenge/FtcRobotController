@file:Suppress("MemberVisibilityCanBePrivate")

package org.firstinspires.ftc.teamcodekt.opmodes.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.outoftheboxrobotics.photoncore.PhotonCore
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import ftc.rogue.blacksmith.Anvil
import ftc.rogue.blacksmith.BlackOp
import ftc.rogue.blacksmith.Scheduler
import ftc.rogue.blacksmith.units.GlobalUnits
import ftc.rogue.blacksmith.util.kt.LateInitVal
import org.checkerframework.checker.units.qual.A
import org.firstinspires.ftc.teamcode.AutoData
import org.firstinspires.ftc.teamcodekt.components.*
import org.firstinspires.ftc.teamcodekt.components.meta.AutoBotComponents
import org.firstinspires.ftc.teamcodekt.components.meta.createAutoBotComponents
import kotlin.properties.Delegates

abstract class RogueBaseAuto : BlackOp() {
//    protected val bot by evalOnGo(::createAutoBotComponents)
    protected var bot by LateInitVal<AutoBotComponents>()
    protected var signalID by Delegates.notNull<Int>()

    protected abstract val startPose: Pose2d
    protected abstract fun mainTraj(startPose: Pose2d): Anvil

    final override fun go() {
        bot = createAutoBotComponents()

        PhotonCore.enable()

        val startTraj = mainTraj(startPose)
        Anvil.startAutoWith(startTraj).onSchedulerLaunch()

        bot.camera.update()
        signalID = bot.camera.waitForStartWithVision(this) ?: 2

        Scheduler.launch(this) {
            bot.updateBaseComponents(0.0)
            bot.drive.update()
            mTelemetry.update()
        }
    }

    companion object {
        const val MAX_CYCLES = 4 // here for temp legacy reasons

        const val NUM_CYCLES = 5

        const val LAST_CYCLE = NUM_CYCLES - 1

        @JvmStatic
        protected val liftOffsets = intArrayOf(
            AutoData.AUTO_INTAKE_LIFT_HEIGHT_1,
            AutoData.AUTO_INTAKE_LIFT_HEIGHT_2,
            AutoData.AUTO_INTAKE_LIFT_HEIGHT_3,
            AutoData.AUTO_INTAKE_LIFT_HEIGHT_4,
            AutoData.AUTO_INTAKE_LIFT_HEIGHT_5,
        )
    }
}
