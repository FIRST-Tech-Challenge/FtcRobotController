@file:Suppress("PARAMETER_NAME_CHANGED_ON_OVERRIDE")

package org.firstinspires.ftc.teamcodekt.components.chains

import org.firstinspires.ftc.teamcodekt.blacksmith.chains.CancellableChain
import org.firstinspires.ftc.teamcodekt.blacksmith.listeners.Listener
import org.firstinspires.ftc.teamcodekt.blacksmith.listeners.Timer
import org.firstinspires.ftc.teamcodekt.components.TeleOpBotComponents

class BackwardsDepositChain(val bot: TeleOpBotComponents, clawOpeningTime: Long) : CancellableChain {
    private val depositTimer = Timer(clawOpeningTime)
    private val liftTimer = Timer(200)

    private var isCancelled = false

    override fun invokeOn(button: Listener) {
        button
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
                    bot.arm.setToBackwardsPos()
                    bot.wrist.setToBackwardsPos()
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
