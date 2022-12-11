@file:Suppress("MemberVisibilityCanBePrivate")

package org.firstinspires.ftc.teamcodekt.opmodes.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcodekt.blacksmith.Scheduler
import org.firstinspires.ftc.teamcodekt.blacksmith.listeners.ReforgedGamepad
import org.firstinspires.ftc.teamcodekt.components.*
import org.firstinspires.ftc.teamcodekt.components.chains.BackwardsDepositChain
import org.firstinspires.ftc.teamcodekt.components.chains.ForwardsDepositChain
import org.firstinspires.ftc.teamcodekt.components.chains.IntakeChain

abstract class RougeBaseTele : LinearOpMode() {
    protected val driver   = ReforgedGamepad(gamepad1)
    protected val codriver = ReforgedGamepad(gamepad2)

    protected var powerMulti  = 0.0

    protected val bot = createTeleOpBotComponents(hardwareMap)

    protected val intakeChain = IntakeChain(bot, 200)
    protected val forwardsDepositChain = ForwardsDepositChain(bot, 500)
    protected val backwardsDepositChain = BackwardsDepositChain(bot, 500)

    override fun runOpMode() = with(bot) {
        waitForStart()

        Scheduler.beforeEach {
            arm.setToRestingPos()
            wrist.setToRestingPos()
            powerMulti = 1.0
        }

        describeControls()

        Scheduler.start(this@RougeBaseTele) {
            lift.update()
            wrist.update()
            telemetry.update()
        }
    }

    abstract fun describeControls()
}