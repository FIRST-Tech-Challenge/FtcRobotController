@file:Suppress("MemberVisibilityCanBePrivate")

package org.firstinspires.ftc.teamcodekt.opmodes.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import ftc.rouge.blacksmith.Scheduler
import ftc.rouge.blacksmith.chains.CancellableChain
import ftc.rouge.blacksmith.chains.Chain
import ftc.rouge.blacksmith.listeners.ReforgedGamepad
import org.firstinspires.ftc.teamcodekt.components.*
import org.firstinspires.ftc.teamcodekt.components.chains.BackwardsDepositChain
import org.firstinspires.ftc.teamcodekt.components.chains.ForwardsDepositChain
import org.firstinspires.ftc.teamcodekt.components.chains.IntakeChain
import ftc.rouge.blacksmith.util.kt.LateInitVal

abstract class RougeBaseTele : LinearOpMode() {
    protected var driver   by LateInitVal<ReforgedGamepad>()
    protected var codriver by LateInitVal<ReforgedGamepad>()

    protected var powerMulti  = 0.0

    protected var bot by LateInitVal<TeleOpBotComponents>()

    protected var intakeChain by LateInitVal<Chain>()
    protected var forwardsDepositChain by LateInitVal<CancellableChain>()
    protected var backwardsDepositChain by LateInitVal<CancellableChain>()

    override fun runOpMode() {
        driver   = ReforgedGamepad(gamepad1)
        codriver = ReforgedGamepad(gamepad2)

        bot = createTeleOpBotComponents(hardwareMap, VoltageScaler(hardwareMap))

        intakeChain = IntakeChain(bot)
        forwardsDepositChain = ForwardsDepositChain(bot)
        backwardsDepositChain = BackwardsDepositChain(bot)

        describeControls()

        waitForStart()

        Scheduler.beforeEach {
            powerMulti = 1.0
        }

        Scheduler.launch(this@RougeBaseTele) {
            bot.updateComponents()
            telemetry.update()
        }
    }

    abstract fun describeControls()
}
