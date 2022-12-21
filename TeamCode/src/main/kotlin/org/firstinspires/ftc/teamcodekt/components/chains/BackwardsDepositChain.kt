@file:Suppress("PARAMETER_NAME_CHANGED_ON_OVERRIDE")

package org.firstinspires.ftc.teamcodekt.components.chains

import ftc.rouge.blacksmith.chains.CancellableChain
import ftc.rouge.blacksmith.listeners.Listener
import ftc.rouge.blacksmith.listeners.Timer
import ftc.rouge.blacksmith.listeners.after
import org.firstinspires.ftc.teamcodekt.components.TeleOpBotComponents

class BackwardsDepositChain(val bot: TeleOpBotComponents) : CancellableChain {
    private var isCancelled = false

    override fun invokeOn(button: Listener) = button
        .onRise {
            isCancelled = false

            bot.arm.setToBackwardsPos()
            bot.wrist.setToBackwardsPos()
        }
        .onFall {
            if (isCancelled) {
                finish()
                return@onFall
            }

            bot.claw.openForDeposit()

            after(400).milliseconds {
                finish()
            }

            after(600).milliseconds {
                bot.lift.goToZero()
            }
        }
        .hook()

    override fun cancelOn(button: Listener) {
        button.onRise { isCancelled = true }
    }

    private fun finish() {
        bot.claw.close()
        bot.arm.setToRestingPos()
        bot.wrist.setToRestingPos()
    }
}
