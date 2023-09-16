package org.firstinspires.ftc.teamcode.Autonomous

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.robotcore.external.tfod.Recognition
import org.firstinspires.ftc.teamcode.DriveMethods
import org.firstinspires.ftc.teamcode.Variables

@Autonomous(name = "TFODAutoTest1", group = "A")
class TFODAutoTest1 : DriveMethods() {
    override fun runOpMode() {
        // Initialize TensorFlow
        initVision(Variables.VisionProcessors.TFOD)

        telemetry.addLine("Initialized")
        telemetry.update()
        waitForStart()

        while (opModeIsActive()) {
            val recognitions: List<Recognition> = tfod.recognitions

            telemetry.addData("# Objects Detected", recognitions.size)

            for (recognition in recognitions) {
                val x = ((recognition.left + recognition.right) / 2).toDouble()
                val y = ((recognition.top + recognition.bottom) / 2).toDouble()

                telemetry.addData("", " ")
                telemetry.addData(
                    "Image",
                    "%s (%.0f %% Conf.)",
                    recognition.label,
                    recognition.confidence * 100
                )
                telemetry.addData("- Position", "%.0f / %.0f", x, y)
                telemetry.addData("- Size", "%.0f x %.0f", recognition.width, recognition.height)
            }

            telemetry.update()

            sleep(50)
        }
    }
}