@file:Suppress("PARAMETER_NAME_CHANGED_ON_OVERRIDE")

package org.firstinspires.ftc.teamcodekt.components.chains

import ftc.rouge.blacksmith.chains.Chain
import ftc.rouge.blacksmith.listeners.Listener
import ftc.rouge.blacksmith.listeners.Timer
import ftc.rouge.blacksmith.listeners.after
import org.firstinspires.ftc.teamcodekt.components.TeleOpBotComponents

class IntakeChain(val bot: TeleOpBotComponents) : Chain {
    override fun invokeOn(button: Listener) = button
        .onRise {
            bot.claw.openForIntakeNarrow()
            bot.intake.enable()

            bot.arm.setToBackwardsPos()
            bot.wrist.setToBackwardsPos()

            bot.lift.goToZero()
        }
        .onFall {
            bot.claw.close()
            bot.intake.disable()

            bot.arm.setToRestingPos()
            bot.wrist.setToRestingPos()
        }
        .hook()
}
