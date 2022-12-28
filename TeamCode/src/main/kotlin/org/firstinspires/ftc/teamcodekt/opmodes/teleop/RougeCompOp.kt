package org.firstinspires.ftc.teamcodekt.opmodes.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import ftc.rouge.blacksmith.listeners.Listener
import ftc.rouge.blacksmith.util.kt.pow
import org.firstinspires.ftc.teamcodekt.components.LiftConfig

@TeleOp
class RougeCompOp : RougeBaseTele() {
    override fun describeControls() {
        describeCodriverControls()
        describeDriverControls()
    }

    private fun describeDriverControls() = with(bot) {
        Listener { lift.height > LiftConfig.MID * 1.01 }
            .whileHigh { powerMulti /= 2 }

        driver.right_trigger(.1).whileHigh {
            powerMulti *= 1 - driver.right_trigger()
        }

        Listener.always {
            drivetrain.drive(driver.gamepad, powerMulti)
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
            if (codriver.left_stick_x() > .5) {
                claw.openForIntakeWide()
                intake.enable()
            }

            if (codriver.left_stick_x() < -.5) {
                intake.disable()
                claw.close()
            }
        }

        // -- MANUAL LIFT CONTROLS --

        codriver.right_bumper.whileHigh {
            lift.height += 8
        }

        codriver.left_bumper.whileHigh {
            lift.height -= 8
        }
    }
}
