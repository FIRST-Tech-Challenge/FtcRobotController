@file:Suppress("PARAMETER_NAME_CHANGED_ON_OVERRIDE")

package org.firstinspires.ftc.teamcodekt.components.chains

import ftc.rogue.blacksmith.chains.Chain
import ftc.rogue.blacksmith.listeners.Listener
import ftc.rogue.blacksmith.listeners.after
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcodekt.components.TeleOpBotComponents

class IntakeChain(val bot: TeleOpBotComponents) : Chain {
    private var isRunning = false

    override fun invokeOn(button: Listener) {
        button.onRise {
            bot.claw.openForIntakeNarrow()
            bot.intake.enable()

            bot.arm.setToBackwardsPos()
            bot.wrist.setToBackwardsPos()

            bot.lift.goToZero()
            isRunning = true
        }

        button.onFall {
            if (!isRunning) {
                return@onFall
            }

            bot.intake.disable()
            bot.claw.close()

            isRunning = false
            kill()
        }

        Listener { isRunning && bot.rcs.getDistance(DistanceUnit.CM) < .8 }.onRise {
            bot.intake.disable()
            isRunning = false

            after(75).milliseconds {
                bot.claw.close()
            }

            after(250).milliseconds {
                kill()
            }
        }
    }

    private fun kill() {
        bot.arm.setToRestingPos()
        bot.wrist.setToRestingPos()
    }
}
