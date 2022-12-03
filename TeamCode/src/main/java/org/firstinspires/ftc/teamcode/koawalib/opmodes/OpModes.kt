package org.firstinspires.ftc.teamcode.koawalib.opmodes

import com.asiankoala.koawalib.util.Alliance
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp(name = "\uD83E\uDD76")
class KBlueOp : KTeleOp(Alliance.BLUE)

@TeleOp(name = "\uD83E\uDD75")
class KRedOp : KTeleOp(Alliance.RED)

//@Autonomous(name = "\uD83E\uDD76 CLOSE")
//class BlueCloseAuto : KAuto(Alliance.BLUE, true)
//
//@Autonomous(name = "\uD83E\uDD76 FAR")
//class BlueFarAuto : KAuto(Alliance.BLUE, false)
//
//@Autonomous(name = "\uD83E\uDD75 CLOSE")
//class RedCloseAuto : KAuto(Alliance.RED, true)
//
//@Autonomous(name = "\uD83E\uDD75 FAR")
//class RedFarAuto : KAuto(Alliance.RED, false)