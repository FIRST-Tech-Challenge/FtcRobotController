@file:Suppress("MemberVisibilityCanBePrivate")

package org.firstinspires.ftc.teamcodekt.opmodes.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import ftc.rouge.blacksmith.Scheduler
import ftc.rouge.blacksmith.listeners.ReforgedGamepad
import org.firstinspires.ftc.teamcodekt.components.*
import org.firstinspires.ftc.teamcodekt.components.chains.BackwardsDepositChain
import org.firstinspires.ftc.teamcodekt.components.chains.ForwardsDepositChain
import org.firstinspires.ftc.teamcodekt.components.chains.IntakeChain
import org.firstinspires.ftc.teamcodekt.util.LateInitVal

abstract class RougeBaseTele : LinearOpMode() {
    protected var driver   by LateInitVal<ReforgedGamepad>()
    protected var codriver by LateInitVal<ReforgedGamepad>()

    protected var powerMulti  = 0.0

    protected var bot by LateInitVal<TeleOpBotComponents>()

    protected var intakeChain by LateInitVal<IntakeChain>()
    protected var forwardsDepositChain by LateInitVal<ForwardsDepositChain>()
    protected var backwardsDepositChain by LateInitVal<BackwardsDepositChain>()

    override fun runOpMode() {
        driver   = ReforgedGamepad(gamepad1)
        codriver = ReforgedGamepad(gamepad2)

        bot = createTeleOpBotComponents(hardwareMap, VoltageScaler(hardwareMap))

        intakeChain = IntakeChain(bot, 200)
        forwardsDepositChain = ForwardsDepositChain(bot, 500)
        backwardsDepositChain = BackwardsDepositChain(bot)

        describeControls()

        waitForStart()

        Scheduler.beforeEach {
            bot.arm.setToRestingPos()
            bot.wrist.setToRestingPos()
            powerMulti = 1.0
        }

        Scheduler.launch(this@RougeBaseTele) {
            bot.lift.update()
            bot.wrist.update()
            bot.arm.update()
            bot.claw.update()
            telemetry.update()
        }
    }

    abstract fun describeControls()
}
