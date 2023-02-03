@file:Suppress("MemberVisibilityCanBePrivate")

package org.firstinspires.ftc.teamcodekt.opmodes.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.outoftheboxrobotics.photoncore.PhotonCore
import ftc.rogue.blacksmith.Anvil
import ftc.rogue.blacksmith.BlackOp
import ftc.rogue.blacksmith.Scheduler
import ftc.rogue.blacksmith.util.kt.LateInitVal
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
        mTelemetry.addLine("Going")
        mTelemetry.update()

        bot = createAutoBotComponents()

        mTelemetry.addLine("Made components")
        mTelemetry.update()

        PhotonCore.enable()

        mTelemetry.addLine("Enabled photon")
        mTelemetry.update()

        val startTraj = mainTraj(startPose)

        mTelemetry.addLine("Made trajectory")
        mTelemetry.update()

        Anvil.startAutoWith(startTraj).onSchedulerLaunch()

        mTelemetry.addLine("Setuo trajectory")
        mTelemetry.update()

        customWaitForStart()

        mTelemetry.addLine("Staritng auto")
        mTelemetry.update()

        Scheduler.launch(opmode = this) {
            bot.updateBaseComponents(true)
            bot.drive.update()
            mTelemetry.update()
        }
    }

    private fun customWaitForStart() {
        bot.camera.readConeAsync()

        mTelemetry.addLine("Starting reading cone")
        mTelemetry.update()

        var rightWasJustPressed = false
        var leftWasJustPressed  = false
        var upWasJustPressed    = false
        var downWasJustPressed  = false

        var x = 0.0
        var y = 0.0

        while (!opModeIsActive()) {
            signalID = bot.camera.coneReadUpdate() ?: 2

            if (gamepad1.dpad_right && !rightWasJustPressed) {
                x += .25
            }
            rightWasJustPressed = gamepad1.dpad_right

            if (gamepad1.dpad_left && !leftWasJustPressed) {
                x -= .25
            }
            leftWasJustPressed = gamepad1.dpad_left

            if (gamepad1.dpad_up && !upWasJustPressed) {
                y += .25
            }
            upWasJustPressed = gamepad1.dpad_up

            if (gamepad1.dpad_down && !downWasJustPressed) {
                y -= .25
            }
            downWasJustPressed = gamepad1.dpad_down

            val xf = (if (x > 0) "+" else "-") + "%4.2f".format(x.absoluteValue)
            val yf = (if (y > 0) "+" else "-") + "%4.2f".format(y.absoluteValue)

            telemetry.addLine("""
                ↑
                | y = $yf ; ← — → x = $xf 
                ↓
            """.trimIndent())

            telemetry.update()
        }

        poleOffset = Vector2d(x, y)
    }

    companion object {
        const val NUM_CYCLES = 5
        const val LAST_CYCLE = NUM_CYCLES - 1

        @JvmStatic
        protected val
            liftOffsets = intArrayOf(
            AutoData.AUTO_INTAKE_LIFT_HEIGHT_1,
            AutoData.AUTO_INTAKE_LIFT_HEIGHT_2,
            AutoData.AUTO_INTAKE_LIFT_HEIGHT_3,
            AutoData.AUTO_INTAKE_LIFT_HEIGHT_4,
            AutoData.AUTO_INTAKE_LIFT_HEIGHT_5,
        )
    }
}
