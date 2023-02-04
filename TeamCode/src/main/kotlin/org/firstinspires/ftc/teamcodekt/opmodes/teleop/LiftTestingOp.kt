package org.firstinspires.ftc.teamcodekt.opmodes.teleop

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.AutoData.*


@Config
@Disabled
@TeleOp
class LiftTestingOp : RogueBaseTele() {

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

    fun setHeight1(){
        bot.lift.targetHeight = AUTO_INTAKE_LIFT_HEIGHT_1
    }

    fun setHeight2(){
        bot.lift.targetHeight = AUTO_INTAKE_LIFT_HEIGHT_2
    }

    fun setHeight3(){
        bot.lift.targetHeight = AUTO_INTAKE_LIFT_HEIGHT_3
    }

    fun setHeight4(){
        bot.lift.targetHeight = AUTO_INTAKE_LIFT_HEIGHT_4
    }

    fun setHeight5(){
        bot.lift.targetHeight = AUTO_INTAKE_LIFT_HEIGHT_5
    }
}
