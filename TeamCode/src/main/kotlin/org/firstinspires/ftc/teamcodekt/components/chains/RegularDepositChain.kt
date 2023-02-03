@file:Suppress("PARAMETER_NAME_CHANGED_ON_OVERRIDE")

package org.firstinspires.ftc.teamcodekt.components.chains

import ftc.rogue.blacksmith.chains.CancellableChain
import ftc.rogue.blacksmith.listeners.Listener
import ftc.rogue.blacksmith.listeners.after
import org.firstinspires.ftc.teamcodekt.components.ANGLED_LIFT_LOW
import org.firstinspires.ftc.teamcodekt.components.meta.TeleOpBotComponents

class RegularDepositChain(val bot: TeleOpBotComponents) : CancellableChain {
    private var isCancelled = false

    override fun invokeOn(button: Listener) = (button + { bot.lift.targetHeight > (ANGLED_LIFT_LOW - 25) })
        .onRise {
            isCancelled = false
            bot.arm.setToForwardsAngledPos()
            bot.wrist.setToForwardsPos()
        }
        .onFall {
            if (isCancelled) {
                return@onFall
            }

            bot.lift.targetHeight = (bot.lift.targetHeight - 100).coerceAtLeast(0)
            after(50).milliseconds {
                if(bot.lift.targetHeight != ANGLED_LIFT_LOW)
                    bot.arm.setToForwardsPos()
            }
            after(190).milliseconds {
                bot.claw.openForDeposit()
            }

            after(395).milliseconds {
                finish()
            }

            after(425).milliseconds {
                bot.lift.goToZero()
            }
        }
        .hook()

    override fun cancelOn(button: Listener) = (button + { !isCancelled })
        .onRise(::finish)
        .hook()

    private fun finish() {
        bot.claw.close()
        bot.arm.setToRestingPos()
        bot.wrist.setToRestingPos()
        isCancelled = true
    }
}
