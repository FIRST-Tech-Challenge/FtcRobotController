package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.aether.collaborative_multitasking.MultitaskScheduler
import org.firstinspires.ftc.teamcode.mmooover.EncoderTracking
import org.firstinspires.ftc.teamcode.mmooover.Player

@TeleOp
class ExampleAuto: LinearOpMode() {
    override fun runOpMode() {
        val scheduler = MultitaskScheduler()
        val h = Hardware(hardwareMap)
        val enc = EncoderTracking(h)
        val player = Player(
            Player.getPathfileByName("example.bin"),
            scheduler,
            enc,
            mapOf()
        ) {
            nonBlocking {
                radius = 2.0
                angleDelta = 45.deg
            }
        }
        player.requestStart()
        waitForStart()
        while (opModeIsActive()) {
            scheduler.tick()
        }
    }
}