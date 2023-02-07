@file:Suppress("MemberVisibilityCanBePrivate")

package org.firstinspires.ftc.teamcodekt.opmodes.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.outoftheboxrobotics.photoncore.PhotonCore
import ftc.rogue.blacksmith.Anvil
import ftc.rogue.blacksmith.BlackOp
import ftc.rogue.blacksmith.Scheduler
import ftc.rogue.blacksmith.util.SignalEdgeDetector
import ftc.rogue.blacksmith.units.DistanceUnit
import ftc.rogue.blacksmith.util.kt.LateInitVal
import ftc.rogue.blacksmith.util.toCm
import org.firstinspires.ftc.teamcode.AutoData
import org.firstinspires.ftc.teamcodekt.components.*
import org.firstinspires.ftc.teamcodekt.components.meta.AutoBotComponents
import org.firstinspires.ftc.teamcodekt.components.meta.createAutoBotComponents
import kotlin.math.absoluteValue
import kotlin.properties.Delegates

abstract class RogueBaseAuto : BlackOp() {
    protected var bot by LateInitVal<AutoBotComponents>()
    protected var signalID by Delegates.notNull<Int>()

    protected abstract val startPose: Pose2d
    protected abstract fun mainTraj(startPose: Pose2d): Anvil

    protected var poleOffset = Vector2d()
        private set

    final override fun go() {
        bot = createAutoBotComponents()

        PhotonCore.enable()

        readPoleOffset()

        val startTraj = mainTraj(startPose)
        Anvil.startAutoWith(startTraj).onSchedulerLaunch()

        signalID = bot.camera.waitForStartWithVision(this) ?: 2

        Scheduler.launch(opmode = this) {
            bot.updateBaseComponents()
            bot.drive.update()
            mTelemetry.addLine("Pole offset: x->${poleOffset.x}, y->${poleOffset.y}")
            mTelemetry.update()
        }
    }

    private fun readPoleOffset() {
        mTelemetry.addLine("Starting reading cone")
        mTelemetry.update()

        val rightSED = SignalEdgeDetector { gamepad1.dpad_right }
        val leftSED  = SignalEdgeDetector { gamepad1.dpad_left  }
        val upSED    = SignalEdgeDetector { gamepad1.dpad_up    }
        val downSED  = SignalEdgeDetector { gamepad1.dpad_down  }

        var x = 0.0
        var y = 0.0

        while (!gamepad1.a) {
            if (rightSED.risingEdge()) {
                x += .25
            }
            rightSED.update()

            if (leftSED.risingEdge()) {
                x -= .25
            }
            leftSED.update()

            if (upSED.risingEdge()) {
                y += .25
            }
            upSED.update()

            if (downSED.risingEdge()) {
                y -= .25
            }
            downSED.update()

            val xf = (if (x > 0) "+" else "-") + "%4.2f".format(x.absoluteValue)
            val yf = (if (y > 0) "+" else "-") + "%4.2f".format(y.absoluteValue)

            telemetry.addLine("""
                ↑
                | y = $yf ; ← — → x = $xf 
                ↓
            """.trimIndent())

            telemetry.update()
        }

        poleOffset = Vector2d(x.toCm(DistanceUnit.INCHES), y.toCm(DistanceUnit.INCHES))
    }

    companion object {
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
