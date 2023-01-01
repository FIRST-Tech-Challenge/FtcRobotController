package org.firstinspires.ftc.teamcodekt.opmodes.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import ftc.rogue.blacksmith.listeners.Listener
import org.firstinspires.ftc.teamcodekt.components.LiftConfig

@TeleOp
class RogueTestingOp : RogueBaseTele() {
    override fun describeControls(): Unit = with(bot) {
        driver.dpad_up   .onRise(lift::goToHigh)
        driver.dpad_down .onRise(lift::goToZero)
        driver.dpad_right.onRise(lift::goToMid)
        driver.dpad_left .onRise(lift::goToLow)

        intakeChain.invokeOn(driver.left_bumper)

        regularDepositChain.invokeOn(driver.right_bumper)
        regularDepositChain.cancelOn(driver.x)

        reverseDepositChain.invokeOn(driver.y)
        reverseDepositChain.cancelOn(driver.x)

        driver.right_trigger(.1).whileHigh {
            powerMulti *= 1 - driver.right_trigger()
        }

        driver.left_trigger.whileHigh {
            if (driver.right_stick_y() > .1) {
                lift.height += (50 * -driver.right_stick_y()).toInt()
            }

            if (driver.left_stick_x() < -.5) {
                claw.openForIntakeWide()
            }

            if (driver.left_stick_x() > .5) {
                claw.close()
            }
        }

        Listener.always {
            bot.drivetrain.drive(driver.gamepad, powerMulti)
        }

        Listener { lift.height > LiftConfig.MID * 1.01 }
            .whileHigh { powerMulti /= 2 }
    }
}
