package org.firstinspires.ftc.teamcodekt.opmodes.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import ftc.rogue.blacksmith.util.kt.pow
import kotlin.math.sign

@TeleOp
class RogueCompOp : RogueBaseTele() {
    private fun describeDriverControls() = with(driver) {
        (left_bumper + right_bumper).onRise {
            bot.drivetrain.resetIMU()
        }

        b.onRise(bot.drivetrain::switchMode)

        right_trigger(deadzone = .2).whileHigh {
            powerMulti = 0.3825 * (driver.right_trigger() pow .825)
        }

        left_trigger.whileHigh {
            powerMulti = -0.1 // Lets robot stop on a dime
        }
    }

    private fun describeCodriverControls() = with(codriver) {
        dpad_up   .onRise(bot.lift::goToHigh)
        dpad_down .onRise(bot.lift::goToZero)
        dpad_right.onRise(bot.lift::goToMid)
        dpad_left .onRise(bot.lift::goToLow)

        // -- TASK CHAINS --

        intakeChain.invokeOn(left_trigger)

        regularDepositChain.invokeOn(right_trigger)
        regularDepositChain.cancelOn(x)

        reverseDepositChain.invokeOn(y)
        reverseDepositChain.cancelOn(x)

        coneUnflipperChain.invokeOn(a)

        // -- MANUAL CLAW CONTROLS --

        left_stick_x.whileHigh {
            if (left_stick_x().sign < 0) {
                bot.claw.openForIntakeWide()
                bot.intake.enable()
            }

            if (left_stick_x().sign > 0) {
                bot.intake.disable()
                bot.claw.close()
            }
        }

        // -- MANUAL LIFT CONTROLS --

        val bumpersPressed = left_bumper + right_bumper

        (right_stick_y(deadzone = .1) + !bumpersPressed).whileHigh {
            bot.lift.targetHeight += (-codriver.right_stick_y() * 10).toInt()
        }

        // -- MANUAL LIFT RESET --

        (bumpersPressed + right_stick_y(deadzone = .1)).whileHigh {
            bot.lift.targetHeight += (-codriver.right_stick_y() * 2.5).toInt()
        }

        bumpersPressed.onFall {
            bot.lift.resetEncoder()
        }
    }

    override fun describeControls() {
        describeCodriverControls()
        describeDriverControls()
    }
}
