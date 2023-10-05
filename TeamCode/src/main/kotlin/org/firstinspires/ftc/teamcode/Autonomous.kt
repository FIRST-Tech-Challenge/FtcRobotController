package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode

@Autonomous(name = "Kotlin Auto (Blue Left)", group = "Kt")
class AutoBlueLeft: AutoSuper(AutoSuper.Alliance.BLUE, AutoSuper.AllianceSide.LEFT)
@Autonomous(name = "Kotlin Auto (Blue Right)", group = "Kt")
class AutoBlueRight: AutoSuper(AutoSuper.Alliance.BLUE, AutoSuper.AllianceSide.RIGHT)
@Autonomous(name = "Kotlin Auto (Red Left)", group = "Kt")
class AutoRedLeft: AutoSuper(AutoSuper.Alliance.RED, AutoSuper.AllianceSide.LEFT)
@Autonomous(name = "Kotlin Auto (Red Right)", group = "Kt")
class AutoRedRight: AutoSuper(AutoSuper.Alliance.RED, AutoSuper.AllianceSide.RIGHT)

//@Disabled
open class AutoSuper(val alliance: Alliance, val side: AllianceSide) : OpMode() {
    enum class Alliance {
        RED,
        BLUE
    }
    enum class AllianceSide {
        LEFT,
        RIGHT
    }

    private lateinit var shared: BotShared

    override fun init() {
        shared = BotShared(this)
    }

    override fun loop() { }

    override fun start() {
        TODO("Not yet implemented")
        // Game Plan:
        // - Place a purple pixel on the spike mark
        // - Do other stuff (we're working on it!)
    }

}