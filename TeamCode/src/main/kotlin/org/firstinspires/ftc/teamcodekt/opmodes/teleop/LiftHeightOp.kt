package org.firstinspires.ftc.teamcodekt.opmodes.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp
class LiftHeightOp : RogueBaseTele() {
    override fun describeControls(): Unit = with(bot) {
//        driver.dpad_up   .onRise(lift::incPower)
//        driver.dpad_down .onRise(lift::decPower)
    }
}
