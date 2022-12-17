@file:Suppress("PARAMETER_NAME_CHANGED_ON_OVERRIDE")

package org.firstinspires.ftc.teamcodekt.components.chains

import ftc.rouge.blacksmith.chains.Chain
import ftc.rouge.blacksmith.listeners.Listener
import ftc.rouge.blacksmith.listeners.Timer
import org.firstinspires.ftc.teamcodekt.components.TeleOpBotComponents

class IntakeChain(val bot: TeleOpBotComponents, clawClosingTime: Long) : Chain {
    private var clawTimer = Timer(clawClosingTime)

    override fun invokeOn(button: Listener) {
        button
            .onRise {
                bot.intake.enable()
                bot.claw.openForIntakeNarrow()
                bot.lift.goToZero()
                clawTimer.setPending()
            }
            .onFall {
                bot.claw.close()
                bot.intake.disable()
                clawTimer.start()
            }

        clawTimer
            .whileWaiting {
                bot.arm.setToBackwardsPos()
                bot.wrist.setToBackwardsPos()
            }
            .onDone(bot.lift::goToZero)
    }
}
