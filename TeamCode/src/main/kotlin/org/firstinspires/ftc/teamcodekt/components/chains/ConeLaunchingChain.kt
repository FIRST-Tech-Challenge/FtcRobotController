@file:Suppress("PARAMETER_NAME_CHANGED_ON_OVERRIDE")

package org.firstinspires.ftc.teamcodekt.components.chains

import com.acmerobotics.dashboard.config.Config
import ftc.rogue.blacksmith.chains.CancellableChain
import ftc.rogue.blacksmith.listeners.Listener
import ftc.rogue.blacksmith.listeners.after
import org.firstinspires.ftc.teamcodekt.components.meta.TeleOpBotComponents

@Config
class ConeLaunchingChain(val bot: TeleOpBotComponents) : CancellableChain {
    private var isCancelled = false

    override fun invokeOn(button: Listener) = (button)
        .onRise {
            isCancelled = false
            bot.arm.targetAngle = 25.0
            bot.wrist.setToForwardsPos()
        }
        .onFall {
            if (isCancelled) {
                return@onFall
            }

            bot.arm.targetAngle = 145.0

            after(WAITING_TIME_B4_LAUNCH).milliseconds {
                bot.intake.reverse()
                bot.claw.openForIntakeWide()
            }

            after(WAITING_TIME_B4_RESET).milliseconds {
                finish()
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
        bot.intake.disable()

        isCancelled = true

        after(65).milliseconds {
            bot.lift.goToZero()
        }
    }

    companion object {
        @JvmField var WAITING_TIME_B4_LAUNCH = 245L
        @JvmField var WAITING_TIME_B4_RESET = WAITING_TIME_B4_LAUNCH + 125L
    }
}
