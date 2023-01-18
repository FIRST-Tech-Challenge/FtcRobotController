@file:Suppress("MemberVisibilityCanBePrivate")

package org.firstinspires.ftc.teamcodekt.opmodes.auto

import com.outoftheboxrobotics.photoncore.PhotonCore
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import ftc.rogue.blacksmith.BlackOp
import ftc.rogue.blacksmith.util.kt.LateInitVal
import org.checkerframework.checker.units.qual.A
import org.firstinspires.ftc.teamcode.AutoData
import org.firstinspires.ftc.teamcodekt.components.*
import org.firstinspires.ftc.teamcodekt.components.meta.AutoBotComponents
import org.firstinspires.ftc.teamcodekt.components.meta.createAutoBotComponents

abstract class RogueBaseAuto : BlackOp() {
    //    protected val bot by evalOnGo(::createAutoBotComponents)
    protected var bot by LateInitVal<AutoBotComponents>()

    abstract fun executeOrder66()

    final override fun go() {
        bot = createAutoBotComponents()

        quickLog("Starting futon")
        PhotonCore.enable()

        quickLog("Killing all the childen")
        executeOrder66()
    }

    protected fun updateComponents() {
        bot.updateBaseComponents(0.0)
        bot.drive.update()
        mTelemetry.update()
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
