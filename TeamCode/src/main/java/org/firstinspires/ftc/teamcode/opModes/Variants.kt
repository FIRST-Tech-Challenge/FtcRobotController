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

@Autonomous
class RedGardenAuto: GardenAuto(true)

@Autonomous
class BlueGardenAuto: GardenAuto(false)
@Autonomous
class RedDumbAuto: DumbAuto(true)

@Autonomous
class BlueDumbAuto: DumbAuto(false)