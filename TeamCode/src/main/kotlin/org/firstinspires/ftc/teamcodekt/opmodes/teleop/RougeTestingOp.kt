package org.firstinspires.ftc.teamcodekt.opmodes.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcodekt.blacksmith.listeners.Listener
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

        driver.left_trigger(.1).whileHigh {
            powerMulti /= 2
        }

        Listener { lift.height > LiftConfig.MID * 1.001 }
            .whileHigh { powerMulti /= 2 }
    }
}
