@file:Suppress("MemberVisibilityCanBePrivate")

package org.firstinspires.ftc.teamcodekt.opmodes.teleop

import com.outoftheboxrobotics.photoncore.PhotonCore
import ftc.rouge.blacksmith.BlackOp
import ftc.rouge.blacksmith.Scheduler
import ftc.rouge.blacksmith.listeners.ReforgedGamepad
import ftc.rouge.blacksmith.util.kt.createOnGo
import org.firstinspires.ftc.teamcodekt.components.*
import org.firstinspires.ftc.teamcodekt.components.chains.IntakeChain
import org.firstinspires.ftc.teamcodekt.components.chains.RegularDepositChain
import org.firstinspires.ftc.teamcodekt.components.chains.ReverseDepositChain

abstract class RougeBaseTele : BlackOp() {
    protected val driver   by createOnGo<ReforgedGamepad>({ gamepad1 })
    protected val codriver by createOnGo<ReforgedGamepad>({ gamepad2 })

    protected var powerMulti = 0.0

    protected val bot by createOnGo {
        createTeleOpBotComponents(hwMap, VoltageScaler( hwMap ))
    }

    protected val intakeChain by createOnGo<IntakeChain>({ bot })
    protected val regularDepositChain by createOnGo<RegularDepositChain>({ bot })
    protected val reverseDepositChain by createOnGo<ReverseDepositChain>({ bot })

    final override fun go() {
        PhotonCore.enable()

        describeControls()

        Scheduler.beforeEach {
            powerMulti = 1.0
        }

        Scheduler.launchWhenReady(this@RougeBaseTele) {
            bot.updateComponents(mTelemetry)
            mTelemetry.update()
        }
    }

    abstract fun describeControls()
}
