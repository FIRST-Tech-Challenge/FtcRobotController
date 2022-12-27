@file:Suppress("MemberVisibilityCanBePrivate")

package org.firstinspires.ftc.teamcodekt.opmodes.teleop

import com.arcrobotics.ftclib.hardware.motors.Motor
import ftc.rouge.blacksmith.BlackOp
import ftc.rouge.blacksmith.Scheduler
import ftc.rouge.blacksmith.listeners.ReforgedGamepad
import ftc.rouge.blacksmith.util.kt.CreateOnStartR
import ftc.rouge.blacksmith.util.kt.createOnStart
import org.firstinspires.ftc.teamcodekt.components.*
import org.firstinspires.ftc.teamcodekt.components.chains.IntakeChain
import org.firstinspires.ftc.teamcodekt.components.chains.RegularDepositChain
import org.firstinspires.ftc.teamcodekt.components.chains.ReverseDepositChain

abstract class RougeBaseTele : BlackOp() {
    protected val driver   by createOnStart<ReforgedGamepad>({ gamepad1 })
    protected val codriver by createOnStart<ReforgedGamepad>({ gamepad2 })

    protected var powerMulti = 0.0

    protected val bot by createOnStart {
        createTeleOpBotComponents(hwMap, VoltageScaler( hwMap ))
    }

    protected val intakeChain by createOnStart<IntakeChain>({ bot })
    protected val regularDepositChain by createOnStart<RegularDepositChain>({ bot })
    protected val reverseDepositChain by createOnStart<ReverseDepositChain>({ bot })

    final override fun run() {
        describeControls()

        val fl = Motor(hardwareMap, "FL") // Left Deadwheel
        val br = Motor(hardwareMap, "BR") // Right Deadhweel
        val bl = Motor(hardwareMap, "BL") // Horizontal deadwheel

        fl.resetEncoder()
        br.resetEncoder()
        bl.resetEncoder()

        waitForStart()

        Scheduler.beforeEach {
            powerMulti = 1.0
        }

        Scheduler.launch(this@RougeBaseTele) {
            bot.updateComponents(mTelemetry)
            mTelemetry.addData("Left Deadwheel", fl.currentPosition)
            mTelemetry.addData("Right Deadwheel", br.currentPosition)
            mTelemetry.addData("Horizontal Deadwheel", bl.currentPosition)
            mTelemetry.update()
        }
    }

    abstract fun describeControls()
}
