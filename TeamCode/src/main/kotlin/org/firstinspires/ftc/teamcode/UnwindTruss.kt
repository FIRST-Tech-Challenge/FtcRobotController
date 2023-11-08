package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.clamp
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

@TeleOp(name = "Unwind Truss")
class UnwindTruss : OpMode() {

    private lateinit var shared: BotShared

    override fun init() {
        shared = BotShared(this)
    }

    override fun start() {
        super.start()
        shared.motorTruss?.power = gamepad1.right_trigger.toDouble() - gamepad1.right_trigger.toDouble()
    }

    override fun loop() {
        shared.update()
    }
}