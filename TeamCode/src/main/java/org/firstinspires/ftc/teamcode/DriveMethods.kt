package org.firstinspires.ftc.teamcode

import android.util.Size
import com.google.blocks.ftcrobotcontroller.util.CurrentGame
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.TouchSensor
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
import org.firstinspires.ftc.teamcode.Variables.motorSlideLeft
import org.firstinspires.ftc.teamcode.Variables.motorSlideRotate
import org.firstinspires.ftc.teamcode.Variables.rMotorL
import org.firstinspires.ftc.teamcode.Variables.rMotorR
import org.firstinspires.ftc.teamcode.Variables.slideAngle
import org.firstinspires.ftc.teamcode.Variables.slideGate
import org.firstinspires.ftc.teamcode.Variables.slideLength
import org.firstinspires.ftc.teamcode.Variables.t
import org.firstinspires.ftc.teamcode.Variables.targetFound
import org.firstinspires.ftc.teamcode.Variables.touchyL
import org.firstinspires.ftc.teamcode.Variables.touchyR
import org.firstinspires.ftc.teamcode.Variables.x
import org.firstinspires.ftc.teamcode.Variables.y
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import org.firstinspires.ftc.vision.tfod.TfodProcessor
import java.io.BufferedReader
import java.io.FileReader
import java.util.concurrent.TimeUnit
import kotlin.math.abs
import kotlin.math.atan
import kotlin.math.pow
import kotlin.math.sqrt


open class DriveMethods: LinearOpMode() {
    override fun runOpMode() {}

    lateinit var visionPortal: VisionPortal

    lateinit var tfod: TfodProcessor

    lateinit var aprilTag: AprilTagProcessor

    lateinit var visionProcessor: VisionProcessors

    fun initVision(processorType: VisionProcessors) {
        initVision(processorType, 1.0, "/sdcard/FIRST/models/ssd_mobilenet_v2_320x320_coco17_tpu_8.tflite", readLabels("/sdcard/FIRST/models/ssd_mobilenet_v2_label_map.txt"))
    }

    fun initVision(processorType: VisionProcessors, useRightCam: Boolean) {
        initVision(processorType, 1.0, "/sdcard/FIRST/models/ssd_mobilenet_v2_320x320_coco17_tpu_8.tflite", readLabels("/sdcard/FIRST/models/ssd_mobilenet_v2_label_map.txt"), useRightCam)
    }

    fun initVision(processorType: VisionProcessors, zoom: Double) {
        initVision(processorType, zoom, "/sdcard/FIRST/models/ssd_mobilenet_v2_320x320_coco17_tpu_8.tflite", readLabels("/sdcard/FIRST/models/ssd_mobilenet_v2_label_map.txt"))
    }

    fun getDetectionsSingleTFOD(): Variables.Detection {
        val webcam = hardwareMap.get(WebcamName::class.java, "Webcam 1")
        // Ensure the Webcam is correct
        if (visionPortal.activeCamera != webcam) visionPortal.activeCamera = webcam
        // Wait for recognitions
        sleep(3000)

        val recognitions = tfod.recognitions

        // Convert it to a Position (real)
        if (includesCup(recognitions)) {
            val cup = getCup(recognitions) ?: return Variables.Detection.UNKNOWN

            return if (cup.right < 214) Variables.Detection.LEFT
            else if (cup.right > 214 && cup.right < 428) Variables.Detection.CENTER
            else if (cup.right > 428) Variables.Detection.RIGHT
            else Variables.Detection.UNKNOWN
        } else {
            return Variables.Detection.UNKNOWN
        }
    }

    fun getDetectionsMultiTFOD(): Variables.Detection {
        val webcamL = hardwareMap.get(WebcamName::class.java, "Webcam 1")
        val webcamR = hardwareMap.get(WebcamName::class.java, "Webcam 2")

        // Ensure we are using Left Cam
        if (visionPortal.activeCamera != webcamL)  visionPortal.activeCamera = webcamL; sleep(1000)

        // Wait for recognitions
        sleep(2000)
        val recognitionsL = tfod.recognitions
        // Switch cams
        visionPortal.activeCamera = webcamR
        sleep(3000)
        val recognitionsR = tfod.recognitions

        sleep(500) // Sharing is caring

        // It works (I guess)
        if (includesCup(recognitionsR)) {
            val cup = getCup(recognitionsR) ?: return Variables.Detection.UNKNOWN
            return if (cup.right < 214) Variables.Detection.CENTER else Variables.Detection.RIGHT
        } else if (includesCup(recognitionsL)) {
            val cup = getCup(recognitionsL) ?: return Variables.Detection.UNKNOWN
            return if (cup.right < 426) Variables.Detection.LEFT else Variables.Detection.CENTER
        } else {
            return Variables.Detection.UNKNOWN
        }
    }

    fun includesCup(recognitions: List<Recognition>): Boolean {
        for (recognition in recognitions) {
            if (recognition.label == "cup") return true
        }

        return false
    }

    fun getCup(recognitions: List<Recognition>): Recognition? {
        for (recognition in recognitions) {
            if (recognition.label == "cup") return recognition
        }
        return null
    }



    fun initVision(processorType: VisionProcessors, zoom: Double = 1.0, model: String = "/sdcard/FIRST/models/ssd_mobilenet_v2_320x320_coco17_tpu_8.tflite", labelMap: Array<String> = readLabels("/sdcard/FIRST/models/ssd_mobilenet_v2_label_map.txt"), useRightCam: Boolean = false) {
        // Create the bob the builder to build the VisionPortal
        val builder: VisionPortal.Builder = VisionPortal.Builder()

        // Set the camera of the new VisionPortal to the webcam mounted to the robot
        builder.setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 1"))

        // Enable Live View for debugging purposes
        builder.enableLiveView(true)
        builder.setCameraResolution(Size(640, 480))

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
                    .setModelLabels(labelMap)
                    .build()

                tfod.setZoom(zoom)
                tfod.setMinResultConfidence(0.3F)
                if (useRightCam) tfod.setZoom(zoom)
                if (useRightCam) tfod.setMinResultConfidence(0.5F)
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

//    fun switchCamera(cameraName: String) {
//        visionPortal.activeCamera = hardwareMap.get(WebcamName::class.java, cameraName)
//        telemetry.addLine()
//    }

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
                telemetry.addData("readLabels()", " %s", label.toString());

                telemetry.update()
            }

            return labelList.toTypedArray()
        } else {
            telemetry.addLine("readLabels() - No Lines read");

            telemetry.update()

            return Array<String>(1) {""}
        }
    }

    fun getDetectionsApriltag(): ArrayList<AprilTagDetection>? {
        return aprilTag.detections
    }

    fun getDetectionsTFOD(): MutableList<Recognition>? {
        return tfod.recognitions
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
        motorFL = hardwareMap.get(DcMotor::class.java, "motorFL")
        motorBL = hardwareMap.get(DcMotor::class.java, "motorBL")
        motorFR = hardwareMap.get(DcMotor::class.java, "motorFR")
        motorBR = hardwareMap.get(DcMotor::class.java, "motorBR")
        rMotorR = hardwareMap.get(DcMotor::class.java, "rMotorR");
        rMotorL = hardwareMap.get(DcMotor::class.java, "rMotorL");
        touchyR = hardwareMap.get(TouchSensor::class.java, "touchyR")
        touchyL = hardwareMap.get(TouchSensor::class.java, "touchyL")
        slideGate = hardwareMap.get(Servo::class.java, "slideGate")
        motorSlideRotate = hardwareMap.get(DcMotor::class.java, "motorSlideRotate")
        motorSlideLeft = hardwareMap.get(DcMotor::class.java, "motorSlideLeft")
    }

    open fun initSlideMotors() {
        motorSlideLeft = hardwareMap.get<DcMotor>(DcMotor::class.java, "motorSlideLeft")
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
            val exposureControl = visionPortal.getCameraControl(
                ExposureControl::class.java
            )
            if (exposureControl.mode != ExposureControl.Mode.Manual) {
                exposureControl.mode = ExposureControl.Mode.Manual
                sleep(50)
            }
            exposureControl.setExposure(exposureMS.toLong(), TimeUnit.MILLISECONDS)
            sleep(20)
            val gainControl = visionPortal.getCameraControl(
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
            else -> telemetry.addLine("Zoom not updated")
        }
    }

    fun linearSlideCalc() {
        x =  Variables.slideToBoard - Variables.clawToBoard + .5* t
        y = sqrt(3.0) /2 * t
        slideLength = sqrt(x.pow(2.0) + y.pow(2.0))
        slideAngle = atan(y/x)
        x =  abs(Variables.slideToBoard) - Variables.clawToBoard + .5* t
        y = sqrt(3.0) /2 * t
        slideLength = sqrt(x.pow(2.0) + y.pow(2.0))
        slideAngle = atan(y/x)
        clawAngle = 60 - slideAngle
    }


}