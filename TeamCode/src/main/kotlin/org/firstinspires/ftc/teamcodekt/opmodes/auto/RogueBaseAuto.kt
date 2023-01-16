@file:Suppress("MemberVisibilityCanBePrivate")

package org.firstinspires.ftc.teamcodekt.opmodes.auto

import com.outoftheboxrobotics.photoncore.PhotonCore
import ftc.rogue.blacksmith.BlackOp
import ftc.rogue.blacksmith.util.kt.LateInitVal
import org.firstinspires.ftc.teamcode.AutoData
import org.firstinspires.ftc.teamcodekt.components.*
import org.firstinspires.ftc.teamcodekt.components.meta.AutoBotComponents
import org.firstinspires.ftc.teamcodekt.components.meta.createAutoBotComponents

abstract class RogueBaseAuto : BlackOp() {
//    protected val bot by evalOnGo(::createAutoBotComponents)
    protected var bot by LateInitVal<AutoBotComponents>()

    abstract fun execute()

    final override fun go() {
        bot = createAutoBotComponents()


        telemetry.addLine("Staarting photon")
        telemetry.update()
        PhotonCore.enable()

        telemetry.addLine("Killing all the childen")
        telemetry.update()
        execute()
    }

    protected fun updateComponents() {
        bot.updateBaseComponents(0.0)
        bot.drive.update()
        mTelemetry.update()
    }

    companion object {
        // Lens intrinsics
        // Units are in pixels
        private const val fx = 578.272
        private const val fy = 578.272
        private const val cx = 402.145
        private const val cy = 221.506

        // UNITS ARE METERS
        private const val tagsize = 0.166
        private const val DECIMATION_HIGH = 3f
        private const val DECIMATION_LOW = 2f
        private const val THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f
        private const val THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4

        @JvmStatic
        protected val MAX_CYCLES = 4 // here for temp legacy reasons

        @JvmStatic
        protected val NUM_CYCLES = 5

        @JvmStatic
        protected val LAST_CYCLE = 4

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
