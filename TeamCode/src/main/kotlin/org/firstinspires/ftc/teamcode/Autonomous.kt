package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

@Autonomous(name = "Kotlin Auto (Blue Left)", group = "Kt")
class KtAutoBlueLeft: KtAutoSuper(KtAutoSuper.Alliance.BLUE, KtAutoSuper.AllianceSide.LEFT) { }
@Autonomous(name = "Kotlin Auto (Blue Right)", group = "Kt")
class KtAutoBlueRight: KtAutoSuper(KtAutoSuper.Alliance.BLUE, KtAutoSuper.AllianceSide.RIGHT) { }
@Autonomous(name = "Kotlin Auto (Red Left)", group = "Kt")
class KtAutoRedLeft: KtAutoSuper(KtAutoSuper.Alliance.RED, KtAutoSuper.AllianceSide.LEFT) { }
@Autonomous(name = "Kotlin Auto (Red Right)", group = "Kt")
class KtAutoRedRight: KtAutoSuper(KtAutoSuper.Alliance.RED, KtAutoSuper.AllianceSide.RIGHT) { }

//@Disabled
open class KtAutoSuper(val alliance: Alliance, val side: AllianceSide) : LinearOpMode() {
    enum class Alliance {
        RED,
        BLUE
    }
    enum class AllianceSide {
        LEFT,
        RIGHT
    }

    /**
     * Shared constructor block
     */
    init {
//        hardware map
    }
    override fun runOpMode() {
        TODO("Not yet implemented")
        // Game Plan:
        // - Place a purple pixel on the spike mark
        // - Do other stuff (we're working on it!)
    }

}