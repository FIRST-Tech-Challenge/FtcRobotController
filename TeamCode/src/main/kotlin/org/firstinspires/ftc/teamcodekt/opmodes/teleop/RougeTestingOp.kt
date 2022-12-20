package org.firstinspires.ftc.teamcodekt.opmodes.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import ftc.rouge.blacksmith.listeners.Listener
import org.firstinspires.ftc.teamcodekt.components.LiftConfig

@TeleOp
class RougeTestingOp : RougeBaseTele() {
    override fun describeControls(): Unit = with(bot) {
        driver.dpad_up   .onRise(lift::goToHigh)
        driver.dpad_down .onRise(lift::goToZero)
        driver.dpad_right.onRise(lift::goToMid)
        driver.dpad_left .onRise(lift::goToLow)

        intakeChain.invokeOn(driver.left_bumper)

        forwardsDepositChain.invokeOn(driver.right_bumper)
        forwardsDepositChain.cancelOn(driver.x)

        backwardsDepositChain.invokeOn(driver.y)
        backwardsDepositChain.cancelOn(driver.x)

        driver.right_trigger(.1).whileHigh {
            powerMulti *= 1 - (driver.right_trigger() * driver.right_trigger() * driver.right_trigger())
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

            bot.drivetrain.drive(driver.gamepad, 0.0)
        }

        driver.left_trigger.whileLow {
            bot.drivetrain.drive(driver.gamepad, powerMulti)
        }

//        Listener { lift.height > LiftConfig.MID * 1.01 }
//            .whileHigh { powerMulti /= 2 }
    }
}
