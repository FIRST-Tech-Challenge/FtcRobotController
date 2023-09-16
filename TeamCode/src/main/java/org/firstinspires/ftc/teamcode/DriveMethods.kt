package org.firstinspires.ftc.teamcode

import com.google.blocks.ftcrobotcontroller.util.CurrentGame
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.tfod.Recognition
import org.firstinspires.ftc.teamcode.Variables.VisionProcessors
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import org.firstinspires.ftc.vision.tfod.TfodProcessor

open class DriveMethods: LinearOpMode() {
    override fun runOpMode() {}

    lateinit var visionPortal: VisionPortal

    lateinit var tfod: TfodProcessor

    lateinit var aprilTag: AprilTagProcessor

    lateinit var visionProcessor: VisionProcessors

    fun initVision(processorType: VisionProcessors) {
        // Create the bob the builder to build the VisionPortal
        val builder: VisionPortal.Builder = VisionPortal.Builder()

        // Set the camera of the new VisionPortal to the webcam mounted to the robot
        builder.setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 1"))

        // Enable Live View for debugging purposes
        builder.enableLiveView(true)

        when (processorType) {
            VisionProcessors.APRILTAG -> {
                // Initialize the AprilTag Processor
                aprilTag = AprilTagProcessor.Builder()
                    .build()

                builder.addProcessor(aprilTag)
            }
            VisionProcessors.TFOD -> {
                // Initialize the TensorFlow Processor
                tfod = TfodProcessor.Builder()
                    .setModelAssetName(CurrentGame.TFOD_MODEL_ASSET)
                    .build()

                // Add TensorFlow Processor to VisionPortal builder
                builder.addProcessor(tfod)
            }
            VisionProcessors.BOTH -> {
                // Initialize the TensorFlow Processor
                tfod = TfodProcessor.Builder()
                    .setModelAssetName(CurrentGame.TFOD_MODEL_ASSET)
                    .build()

                // Initialize the AprilTag Processor
                aprilTag = AprilTagProcessor.Builder()
                    .build()

                builder.addProcessor(tfod)
                builder.addProcessor(aprilTag)
            }
        }

        // Build the VisionPortal and set visionPortal to it
        visionPortal = builder.build()

        visionProcessor = processorType
    }

    fun addDataToTelemetry() {
        when (this.visionProcessor) {
            VisionProcessors.TFOD -> {
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
            }

            VisionProcessors.APRILTAG -> {
                val detectionList: List<AprilTagDetection> = aprilTag.detections

                for (detection in detectionList) {
                    if (detection.metadata != null) {
                        telemetry.addLine(
                            String.format(
                                "\n==== (ID %d) %s",
                                detection.id,
                                detection.metadata.name
                            )
                        )
                        telemetry.addLine(
                            String.format(
                                "XYZ %6.1f %6.1f %6.1f  (inch)",
                                detection.ftcPose.x,
                                detection.ftcPose.y,
                                detection.ftcPose.z
                            )
                        )
                        telemetry.addLine(
                            String.format(
                                "PRY %6.1f %6.1f %6.1f  (deg)",
                                detection.ftcPose.pitch,
                                detection.ftcPose.roll,
                                detection.ftcPose.yaw
                            )
                        )
                        telemetry.addLine(
                            String.format(
                                "RBE %6.1f %6.1f %6.1f  (inch, deg, deg)",
                                detection.ftcPose.range,
                                detection.ftcPose.bearing,
                                detection.ftcPose.elevation
                            )
                        )
                    } else {
                        telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id))
                        telemetry.addLine(
                            String.format(
                                "Center %6.0f %6.0f   (pixels)",
                                detection.center.x,
                                detection.center.y
                            )
                        )
                    }
                }
            }

            else -> telemetry.addLine("Please Initialize Vision")
        }
    }
}