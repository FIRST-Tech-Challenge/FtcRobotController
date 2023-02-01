package org.firstinspires.ftc.teamcodekt.components.chains

import ftc.rogue.blacksmith.chains.Chain
import ftc.rogue.blacksmith.listeners.Listener
import ftc.rogue.blacksmith.listeners.after
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcodekt.components.ARM_BACKWARDS
import org.firstinspires.ftc.teamcodekt.components.meta.TeleOpBotComponents

class ConeUnflipperChain(val bot: TeleOpBotComponents) : Chain {
    private var isRunning = false

    override fun invokeOn(button: Listener) {
        button.onRise {
            bot.claw.openForIntakeNarrow()
            bot.intake.enable()

            bot.lift.targetHeight = 25
            isRunning = true

            after(30).milliseconds {
                bot.arm.targetAngle = ARM_BACKWARDS - 7.5
                bot.wrist.setToRestingPos()
            }
        }

        button.onFall {
            if (!isRunning) {
                return@onFall
            }

            bot.intake.disable()
            bot.claw.close()

            isRunning = false

            bot.wrist.setToRestingPos()
            bot.arm.setToRestingPos()
            bot.lift.goToZero()
        }

        Listener { isRunning && bot.rcs.getDistance(DistanceUnit.CM) < .8 }.onRise {
            bot.intake.disable()
            isRunning = false

            after(15).milliseconds {
                bot.claw.close()
            }

            after(250).milliseconds {
                bot.wrist.setToRestingPos()
                bot.arm.setToRestingPos()
                bot.lift.goToZero()
            }
        }
    }
}
