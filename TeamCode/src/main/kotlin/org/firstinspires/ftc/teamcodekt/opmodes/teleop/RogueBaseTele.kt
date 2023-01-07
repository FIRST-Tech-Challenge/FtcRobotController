@file:Suppress("MemberVisibilityCanBePrivate")

package org.firstinspires.ftc.teamcodekt.opmodes.teleop

import com.outoftheboxrobotics.photoncore.PhotonCore
import ftc.rogue.blacksmith.BlackOp
import ftc.rogue.blacksmith.Scheduler
import ftc.rogue.blacksmith.listeners.ReforgedGamepad
import org.firstinspires.ftc.teamcodekt.components.*
import org.firstinspires.ftc.teamcodekt.components.chains.IntakeChain
import org.firstinspires.ftc.teamcodekt.components.chains.RegularDepositChain
import org.firstinspires.ftc.teamcodekt.components.chains.ReverseDepositChain
import kotlin.math.abs

abstract class RogueBaseTele : BlackOp() {
    protected val driver   by createOnGo<ReforgedGamepad> { gamepad1 }
    protected val codriver by createOnGo<ReforgedGamepad> { gamepad2 }

    protected var powerMulti = 0.0

    protected val bot by evalOnGo(::createTeleOpBotComponents)

    protected val intakeChain by createOnGo<IntakeChain> { bot }
    protected val regularDepositChain by createOnGo<RegularDepositChain> { bot }
    protected val reverseDepositChain by createOnGo<ReverseDepositChain> { bot }

    final override fun go() {
        PhotonCore.enable()

        describeControls()

        Scheduler.beforeEach {
            powerMulti = 1.0
        }

        Scheduler.launchOnStart(this) {
            bot.drivetrain.drive(driver.gamepad, powerMulti)
            bot.updateComponents()
            mTelemetry.update()
        }
    }

    abstract fun describeControls()
}
