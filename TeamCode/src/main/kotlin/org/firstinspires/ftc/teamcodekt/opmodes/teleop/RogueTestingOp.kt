package org.firstinspires.ftc.teamcodekt.opmodes.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp
class RogueTestingOp : RogueBaseTele() {
    override fun describeControls(): Unit = with(bot) {
        driver.dpad_up   .onRise(lift::goToHigh)
        driver.dpad_down .onRise(lift::goToZero)
        driver.dpad_right.onRise(lift::goToMid)
        driver.dpad_left .onRise(lift::goToLow)

        intakeChain.invokeOn(driver.left_trigger)

        regularDepositChain.invokeOn(driver.right_trigger)
        regularDepositChain.cancelOn(driver.x)

        reverseDepositChain.invokeOn(driver.y)
        reverseDepositChain.cancelOn(driver.x)
    }
}
