package org.firstinspires.ftc.teamcode.Teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.DriveMethods
import org.firstinspires.ftc.teamcode.Variables.motorSlideLeft
import org.firstinspires.ftc.teamcode.Variables.motorSlideRight

@TeleOp(name = "LSTest", group = "LinearSlide")
class LinearSlideTest: DriveMethods() {
    override fun runOpMode() {
        initSlideMotors()

        waitForStart()

        var Clicks = 0.0
        while (opModeIsActive()) {
            motorSlideLeft?.power = Clicks
            motorSlideRight?.power = Clicks
            if (gamepad1.a) {
                Clicks += 0.01
            }
            if (gamepad1.b) {
                Clicks -= 0.0   1
            }
            if (gamepad1.y) {
                telemetry.addLine("Clicks: "+Clicks)
                telemetry.update()
            }
        }
    }
}