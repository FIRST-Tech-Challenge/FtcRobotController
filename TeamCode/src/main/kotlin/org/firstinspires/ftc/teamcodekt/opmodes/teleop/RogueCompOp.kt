package org.firstinspires.ftc.teamcodekt.opmodes.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import ftc.rogue.blacksmith.listeners.Listener
import org.firstinspires.ftc.teamcodekt.components.LiftConfig
import org.firstinspires.ftc.teamcodekt.components.getDriveSticks
import kotlin.math.abs
import kotlin.math.sign

@TeleOp
class RogueCompOp : RogueBaseTele() {
    override fun describeControls() {
        describeCodriverControls()
        describeDriverControls()
    }

    private fun describeDriverControls() = with(bot) {
//        Listener { lift.height > LiftConfig.MID * 1.01 }
//            .whileHigh { powerMulti /= 1.5 }

        driver.left_trigger.whileHigh {
            powerMulti = 0.8
        }

        driver.right_trigger.whileHigh {
            powerMulti = 0.45
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

//        codriver.right_bumper.whileHigh {
//            lift.height += 8
//        }
//
//        codriver.left_bumper.whileHigh {
//            lift.height -= 8
//        }

        codriver.right_stick_y(deadzone = .1).whileHigh {
            lift.height += (codriver.right_stick_y() * 10).toInt()
        }
    }
}
