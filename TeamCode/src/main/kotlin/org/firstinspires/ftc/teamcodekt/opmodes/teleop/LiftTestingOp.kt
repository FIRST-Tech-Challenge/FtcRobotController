package org.firstinspires.ftc.teamcodekt.opmodes.teleop

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@Config
@TeleOp
class LiftTestingOp : RogueBaseTele() {
    @JvmField var h1 = 265
    @JvmField var h2 = 205
    @JvmField var h3 = 145
    @JvmField var h4 = 85
    @JvmField var h5 = 5

    override fun describeControls(): Unit = with(bot) {
        driver.a.onRise(::setHeight1)
        driver.b.onRise(::setHeight2)
        driver.x.onRise(::setHeight3)
        driver.y.onRise(::setHeight4)

        (driver.a + driver.b).onRise(::setHeight5)

        driver.left_trigger.onRise(wrist::setToBackwardsPos)

        driver.right_bumper.onRise(arm::setToBackwardsPosButLikeSliiiightlyHigher)
        driver.right_bumper.onFall(arm::setToRestingPos)

        driver.left_bumper.onRise(claw::openForIntakeWide)
        driver.left_bumper.onFall(claw::close)

        (driver.left_bumper + driver.right_bumper).onRise {
            bot.drivetrain.resetIMU()
        }
    }

    private fun setHeight1() {
        bot.lift.targetHeight = h1
    }

    private fun setHeight2() {
        bot.lift.targetHeight = h2
    }

    private fun setHeight3() {
        bot.lift.targetHeight = h3
    }

    private fun setHeight4() {
        bot.lift.targetHeight = h4
    }

    private fun setHeight5() {
        bot.lift.targetHeight = h5
    }
}
