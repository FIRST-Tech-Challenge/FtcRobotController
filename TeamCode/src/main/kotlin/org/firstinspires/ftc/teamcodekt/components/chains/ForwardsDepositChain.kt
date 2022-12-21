@file:Suppress("PARAMETER_NAME_CHANGED_ON_OVERRIDE")

package org.firstinspires.ftc.teamcodekt.components.chains

import ftc.rouge.blacksmith.chains.CancellableChain
import ftc.rouge.blacksmith.listeners.Listener
import ftc.rouge.blacksmith.listeners.Timer
import ftc.rouge.blacksmith.listeners.after
import org.firstinspires.ftc.teamcodekt.components.TeleOpBotComponents

class ForwardsDepositChain(val bot: TeleOpBotComponents) : CancellableChain {
//    private val depositTimer = Timer(500)
//    private val liftTimer    = Timer(200)

    private var isCancelled = false

//    override fun invokeOn(button: Listener) {
//        val liftIsHighEnough = { bot.lift.height > 500 }
//
//        button.and(liftIsHighEnough)
//            .onRise {
//                depositTimer.setPending()
//                isCancelled = false
//            }
//            .onFall {
//                if (!isCancelled) {
//                    bot.claw.openForDeposit()
//                    depositTimer.start()
//                } else {
//                    depositTimer.finishPrematurely()
//                }
//            }
//
//        depositTimer
//            .whileWaiting {
//                if (!isCancelled) {
//                    bot.arm.setToForwardsPos()
//                    bot.wrist.setToForwardsPos()
//                }
//            }
//            .onDone {
//                bot.claw.close()
//                liftTimer.start()
//            }
//
//        liftTimer
//            .onDone(bot.lift::goToZero)
//    }

    override fun invokeOn(button: Listener) = (button + { bot.lift.height > 500 })
        .onRise {
            isCancelled = false

            bot.arm.setToForwardsPos()
            bot.wrist.setToForwardsPos()
        }
        .onFall {
            if (isCancelled) {
                finish()
                return@onFall
            }

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
