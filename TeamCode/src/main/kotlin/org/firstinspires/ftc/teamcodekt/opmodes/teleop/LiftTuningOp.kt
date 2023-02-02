package org.firstinspires.ftc.teamcodekt.opmodes.teleop

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.AutoData.*
import org.firstinspires.ftc.teamcodekt.components.NORMAL_LIFT_D
import org.firstinspires.ftc.teamcodekt.components.NORMAL_LIFT_I
import org.firstinspires.ftc.teamcodekt.components.NORMAL_LIFT_P
import kotlin.math.sign

@Config
@TeleOp
class LiftTuningOp : RogueBaseTele() {
    override fun describeControls(): Unit = with(bot) {
        driver.dpad_up   .onRise(lift::goToHigh)
        driver.dpad_down .onRise(lift::goToZero)
        driver.dpad_right.onRise(lift::goToMid)
        driver.dpad_left .onRise(lift::goToLow)

        driver.x.onRise {
            NORMAL_LIFT_P += .0001
        }

        driver.a.onRise {
            NORMAL_LIFT_P -= .0001
        }

        driver.y.onRise {
            NORMAL_LIFT_D += .0001
        }
        driver.b.onRise {
            NORMAL_LIFT_D -= .0001
        }

        driver.left_bumper.onRise {
            bot.lift.normalPID.setPIDF(NORMAL_LIFT_P, NORMAL_LIFT_I, NORMAL_LIFT_D, 0.0)
        }
    }
}
