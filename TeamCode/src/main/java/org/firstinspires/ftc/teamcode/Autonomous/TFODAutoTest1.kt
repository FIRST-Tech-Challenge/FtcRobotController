package org.firstinspires.ftc.teamcode.Autonomous

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.tfod.Recognition
import org.firstinspires.ftc.teamcode.DriveMethods
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.tfod.TfodProcessor

class TFODAutoTest1 : DriveMethods() {
    private lateinit var visionPortal: VisionPortal;

    private lateinit var tfod: TfodProcessor;

    fun initTfod() {
        // Initialize the TensorFlow Processor
        tfod = TfodProcessor.Builder()
            .build();

        // Create the bob the builder to build the VisionPortal
        var builder: VisionPortal.Builder = VisionPortal.Builder()

        // Set the camera of the new VisionPortal to the webcam mounted to the robot
        builder.setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 1"))

        // Enable Live View for debugging purposes
        builder.enableLiveView(true)

        // Add TensorFlow Processor
        builder.addProcessor(tfod)

        // Build the VisionPortal and set visionPortal to it
        visionPortal = builder.build()
    }
    override fun runOpMode() {
        // Initialize TensorFlow
        initTfod()

        telemetry.addLine("Initialized")
        telemetry.update()
        waitForStart()

        while (opModeIsActive()) {
            var recognitions: List<Recognition> = tfod.recognitions

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
        }
    }
}