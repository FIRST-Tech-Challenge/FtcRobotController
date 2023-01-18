package org.firstinspires.ftc.teamcodekt.opmodes.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlin.math.sign

@TeleOp
class RogueCompOp : RogueBaseTele() {
    override fun describeControls() {
        describeCodriverControls()
        describeDriverControls()
    }

    private fun describeDriverControls() = with(driver) {
        dpad_up   .onRise(bot.lift::goToHigh)
        dpad_down .onRise(bot.lift::goToZero)
        dpad_right.onRise(bot.lift::goToMid)
        dpad_left .onRise(bot.lift::goToLow)

        left_trigger.whileHigh {
            powerMulti = 0.8
        }

        right_trigger.whileHigh {
            powerMulti = 0.275
        }

        (right_trigger + left_trigger).whileHigh {
            powerMulti = -0.1
        }
    }

    private fun describeCodriverControls() = with(bot) {
        codriver.dpad_up   .onRise(lift::goToHigh)
        codriver.dpad_down .onRise(lift::goToZero)
        codriver.dpad_right.onRise(lift::goToMid)
        codriver.dpad_left .onRise(lift::goToLow)

        // -- TASK CHAINS --

        intakeChain.invokeOn(codriver.left_trigger)

        regularDepositChain.invokeOn(codriver.right_trigger)
        regularDepositChain.cancelOn(codriver.x)

        reverseDepositChain.invokeOn(codriver.y)
        reverseDepositChain.cancelOn(codriver.x)

        // -- MANUAL CLAW CONTROLS --

        codriver.left_stick_x.whileHigh {
            if (codriver.left_stick_x().sign < 0) {
                claw.openForIntakeWide()
                intake.enable()
            }

            if (codriver.left_stick_x().sign > 0) {
                intake.disable()
                claw.close()
            }
        }

        // -- MANUAL LIFT CONTROLS --

        codriver.right_stick_y(deadzone = .1).whileHigh {
            lift.height += (-codriver.right_stick_y() * 10).toInt()
        }
    }
}
