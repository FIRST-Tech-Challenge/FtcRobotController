package org.firstinspires.ftc.teamcode

import com.google.blocks.ftcrobotcontroller.util.CurrentGame
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl
import org.firstinspires.ftc.robotcore.external.tfod.Recognition
import org.firstinspires.ftc.teamcode.Variables.VisionProcessors
import org.firstinspires.ftc.teamcode.Variables.clawAngle
import org.firstinspires.ftc.teamcode.Variables.desiredTag
import org.firstinspires.ftc.teamcode.Variables.motorBL
import org.firstinspires.ftc.teamcode.Variables.motorBR
import org.firstinspires.ftc.teamcode.Variables.motorFL
import org.firstinspires.ftc.teamcode.Variables.motorFR
import org.firstinspires.ftc.teamcode.Variables.slideAngle
import org.firstinspires.ftc.teamcode.Variables.slideLength
import org.firstinspires.ftc.teamcode.Variables.targetFound
import org.firstinspires.ftc.teamcode.Variables.x
import org.firstinspires.ftc.teamcode.Variables.y
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import org.firstinspires.ftc.vision.tfod.TfodProcessor
import java.io.BufferedReader
import java.io.FileReader
import java.util.concurrent.TimeUnit


open class DriveMethods: LinearOpMode() {
    override fun runOpMode() {}

    lateinit var visionPortal: VisionPortal

    lateinit var tfod: TfodProcessor

    lateinit var aprilTag: AprilTagProcessor

    lateinit var visionProcessor: VisionProcessors

    fun initVision(processorType: VisionProcessors, zoom: Double = 1.0, model: String = "/sdcard/FIRST/models/ssd_mobilenet_v2_320x320_coco17_tpu_8.tflite") {
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
                    .setModelFileName(model)
                    .build()

                tfod.setZoom(zoom)

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

    private fun readLabels(labelsPath: String): Array<String> {
        val labelList = ArrayList<String>()

        // try to read in the the labels.
        try {
            BufferedReader(FileReader(labelsPath)).use { br ->
                var index = 0
                while (br.ready()) {
                    labelList.add(br.readLine())
                    index++
                }
            }
        } catch (e: Exception) {
            telemetry.addData("Exception", e.localizedMessage)
            telemetry.update()
        }
        if (labelList.size > 0) {
            telemetry.addData("readLabels()", "%d labels read.", labelList.size)
            telemetry.update()

            for (label in labelList) {
                telemetry.addData("readLabels()", " %f", label);

                telemetry.update()
            }

            return labelList.toTypedArray()
        } else {
            telemetry.addLine("readLabels() - No Lines read");

            telemetry.update()

            return Array<String>(1) {""}
        }
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
                    desiredTag = detection
                    targetFound = true
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
    fun moveRobot(x: Double, y: Double, yaw: Double) {
        motorFL!!.power = x - y - yaw
        motorFR!!.power = x + y + yaw
        motorBL!!.power = x + y - yaw
        motorBR!!.power = x - y + yaw
    }
    open fun initMotorsSecondBot() {
        motorFL = hardwareMap.get<DcMotor>(DcMotor::class.java, "motorFL")
        motorBL = hardwareMap.get<DcMotor>(DcMotor::class.java, "motorBL")
        motorFR = hardwareMap.get<DcMotor>(DcMotor::class.java, "motorFR")
        motorBR = hardwareMap.get<DcMotor>(DcMotor::class.java, "motorBR")
    }
    fun setManualExposure(exposureMS: Int, gain: Int) {
        // Wait for the camera to be open, then use the controls
        if (visionPortal == null) {
            return
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal!!.cameraState != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting")
            telemetry.update()
            while (!isStopRequested && visionPortal!!.cameraState != VisionPortal.CameraState.STREAMING) {
                sleep(20)
            }
            telemetry.addData("Camera", "Ready")
            telemetry.update()
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested) {
            val exposureControl = visionPortal!!.getCameraControl(
                ExposureControl::class.java
            )
            if (exposureControl.mode != ExposureControl.Mode.Manual) {
                exposureControl.mode = ExposureControl.Mode.Manual
                sleep(50)
            }
            exposureControl.setExposure(exposureMS.toLong(), TimeUnit.MILLISECONDS)
            sleep(20)
            val gainControl = visionPortal!!.getCameraControl(
                GainControl::class.java
            )
            gainControl.gain = gain
            sleep(20)
        }
    }

    fun quickPrint(message: String) {
        telemetry.addLine(message)
        telemetry.update()
    }

    fun updateZoom(zoom: Double) {
        when (this.visionProcessor) {
            VisionProcessors.TFOD -> tfod.setZoom(zoom)
            VisionProcessors.BOTH -> tfod.setZoom(zoom)

            else -> {}
        }
    }

    fun linearSlideCalc() {
        x =  Variables.slideToBoard - Variables.clawToBoard + .5* Variables.t;
        y = Math.sqrt(3.0)/2 * Variables.t;
        slideLength = Math.sqrt(Math.pow(x, 2.0) + Math.pow(y, 2.0));
        slideAngle = Math.atan(y/x);
        clawAngle = 60 - slideAngle;
    }
}