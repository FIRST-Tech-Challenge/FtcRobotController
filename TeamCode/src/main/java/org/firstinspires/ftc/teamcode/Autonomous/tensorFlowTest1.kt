package org.firstinspires.ftc.teamcode.Autonomous

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.tfod.Recognition
import org.firstinspires.ftc.teamcode.Variables.VisionProcessors
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.tfod.TfodProcessor


@TeleOp(name="Tensor Flow Test", group="Concept")
class tensorFlowTest1: LinearOpMode(){
    private val USE_WEBCAM = true;
    lateinit var tfod: TfodProcessor
    lateinit var visionPortal: VisionPortal

    override fun runOpMode() {
        initTfod();

        waitForStart()

        while (opModeIsActive()) {
            telemetry.addLine("Cool stuff is happening maybe");
            telemetryTfod()
            telemetry.update()
            sleep(100)
        }

    }


    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private fun initTfod() {

        // Create the TensorFlow processor the easy way.
        tfod = TfodProcessor.Builder()
            .setModelFileName("/sdcard/FIRST/models/kollmodel.tflite")
            .build()


        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap[WebcamName::class.java, "Webcam 1"], tfod
            )
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                BuiltinCameraDirection.BACK, tfod
            )
        }
    } // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private fun telemetryTfod() {
        val currentRecognitions: List<Recognition> = tfod.getRecognitions()
        telemetry.addData("# Objects Detected", currentRecognitions.size)

        // Step through the list of recognitions and display info for each one.
        for (recognition in currentRecognitions) {
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
        } // end for() loop
    } // end method telemetryTfod()


}