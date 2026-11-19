package org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.opModes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.TeleOp


@TeleOp
class TeleOpRed: TeleOperation(true)

@TeleOp
class TeleOpBlue : TeleOperation(false)

@Autonomous
class RedAuto : LinearAuto(true)

@Autonomous
class BlueAuto : LinearAuto(false)
