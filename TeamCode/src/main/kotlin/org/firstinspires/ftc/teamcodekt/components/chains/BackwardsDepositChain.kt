@file:Suppress("PARAMETER_NAME_CHANGED_ON_OVERRIDE")

package org.firstinspires.ftc.teamcodekt.components.chains

import org.firstinspires.ftc.teamcodekt.blacksmith.chains.CancellableChain
import org.firstinspires.ftc.teamcodekt.blacksmith.listeners.Listener
import org.firstinspires.ftc.teamcodekt.blacksmith.listeners.Timer
import org.firstinspires.ftc.teamcodekt.components.TeleOpBotComponents

class BackwardsDepositChain(val bot: TeleOpBotComponents) : CancellableChain {
    private val depositTimer = Timer(500)
    private val liftTimer = Timer(200)

    private var isCancelled = false

    override fun invokeOn(button: Listener) {
        val startDepositing = Listener { !button.condition() && !isCancelled }::onRise

        button
            .onRise { isCancelled = false }
            .onFall(bot.claw::openForDeposit)

        depositTimer
            .setPendingOn(button::onRise)
            .startTimerOn(startDepositing)

            .whileWaiting {
                if (isCancelled) {
                    depositTimer.finishPrematurely()
                }

                bot.arm.setToBackwardsPos()
                bot.wrist.setToBackwardsPos()
            }

            .onDone(bot.claw::close)

        liftTimer
            .startTimerOn(depositTimer::onDone)
            .onDone(bot.lift::goToZero)
    }

    override fun cancelOn(button: Listener) {
        button.onRise { isCancelled = true }
    }
}
