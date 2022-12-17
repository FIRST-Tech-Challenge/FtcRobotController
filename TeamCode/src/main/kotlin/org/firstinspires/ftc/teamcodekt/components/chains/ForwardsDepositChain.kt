@file:Suppress("PARAMETER_NAME_CHANGED_ON_OVERRIDE")

package org.firstinspires.ftc.teamcodekt.components.chains

import ftc.rouge.blacksmith.chains.CancellableChain
import ftc.rouge.blacksmith.listeners.Listener
import ftc.rouge.blacksmith.listeners.Timer
import org.firstinspires.ftc.teamcodekt.components.TeleOpBotComponents

class ForwardsDepositChain(val bot: TeleOpBotComponents, clawOpeningTime: Long) : CancellableChain {
    private val depositTimer = Timer(clawOpeningTime)
    private val liftTimer = Timer(200)

    private var isCancelled = false

    override fun invokeOn(button: Listener) {
        val liftIsHighEnough = { bot.lift.height > 500 }

        button.and(liftIsHighEnough)
            .onRise {
                depositTimer.setPending()
                isCancelled = false
            }
            .onFall {
                if (!isCancelled) {
                    bot.claw.openForDeposit()
                    depositTimer.start()
                } else {
                    depositTimer.finishPrematurely()
                }
            }

        depositTimer
            .whileWaiting {
                if (!isCancelled) {
                    bot.arm.setToForwardsPos()
                    bot.wrist.setToForwardsPos()
                }
            }
            .onDone {
                bot.claw.close()
                liftTimer.start()
            }

        liftTimer
            .onDone(bot.lift::goToZero)
    }

    override fun cancelOn(button: Listener) {
        button.onRise { isCancelled = true }
    }
}
