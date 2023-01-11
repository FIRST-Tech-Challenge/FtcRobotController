@file:Suppress("PARAMETER_NAME_CHANGED_ON_OVERRIDE")

package org.firstinspires.ftc.teamcodekt.components.chains

import ftc.rogue.blacksmith.chains.CancellableChain
import ftc.rogue.blacksmith.listeners.Listener
import ftc.rogue.blacksmith.listeners.after
import org.firstinspires.ftc.teamcodekt.components.meta.TeleOpBotComponents

class ReverseDepositChain(val bot: TeleOpBotComponents) : CancellableChain {
    private var isCancelled = false
    private var isRunning = false

    override fun invokeOn(button: Listener) = button
        .onRise {
            isCancelled = false
            isRunning = true

            bot.arm.setToBackwardsPosButLikeSliiiightlyHigher()
            bot.wrist.setToBackwardsPos()
        }
        .onFall {
            if (isCancelled) {
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

    override fun cancelOn(button: Listener) = button
        .onRise {
            isCancelled = true

            if (isRunning) {
                finish()
            }
        }
        .hook()

    private fun finish() {
        bot.claw.close()
        bot.arm.setToRestingPos()
        bot.wrist.setToRestingPos()
        isRunning = false
    }
}
