
package org.firstinspires.ftc.teamcode.Autonomous

import com.google.blocks.ftcrobotcontroller.util.CurrentGame
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.tfod.Recognition
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.tfod.TfodProcessor
import java.io.BufferedReader
import java.io.FileReader
//import org.firstinspires.ftc.vision.tfod.TfodProcessor.Builder.setNumDetectorThreads;

@Disabled
@TeleOp(name="Tensor Flow Test 2", group="Concept")
class tensorFlowTest2: LinearOpMode(){
    private val USE_WEBCAM = true;
    lateinit var tfod: TfodProcessor
    lateinit var tfod2:TfodProcessor
    lateinit var visionPortal: VisionPortal
    var processorToggled = true;

    val tfod_labels = "/sdcard/FIRST/models/ssd_mobilenet_v2_label_map.txt"
    private lateinit var labels: Array<String>
    override fun runOpMode() {
        telemetry.addLine("Reading Lables");
        telemetry.update()
        readLabels();

        telemetry.addLine("Read Lables");
        telemetry.update()

        initTfod();
//        toggleProccessor()
//        makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL)
        telemetry.addLine("TfodsInitiated");
        telemetry.update()
        waitForStart()
//        toggleProccessor();
        while (opModeIsActive()) {
            telemetry.addLine("Cool stuff is happening maybe");
//            if(processorToggled){
            telemetryTfod()
//            }else{
            telemetryTfod2()
//            }
            telemetry.addLine("FPS: "+visionPortal.getFps())
            telemetry.update()
//            toggleProccessor();
            sleep(100)
        }

    }

    fun toggleProccessor(){
      if(processorToggled){
        visionPortal.setProcessorEnabled(tfod, true);
        visionPortal.setProcessorEnabled(tfod2, false);
      }
      else{
      visionPortal.setProcessorEnabled(tfod, false);
      visionPortal.setProcessorEnabled(tfod2, true);
      }
      processorToggled=!processorToggled
    }
    




    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private fun initTfod() {

        // Create the TensorFlow processor the easy way.

        tfod2 = TfodProcessor.Builder()
            .setNumDetectorThreads(3)
            .setModelAssetName(CurrentGame.TFOD_MODEL_ASSET)
            .build()

        tfod = TfodProcessor.Builder()
            .setNumDetectorThreads(3)
            .setModelFileName("/sdcard/FIRST/models/ssd_mobilenet_v2_320x320_coco17_tpu_8.tflite")
            .setModelLabels(labels)
            .build()

        telemetry.addLine("Tfod built");
        telemetry.update()

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.Builder()
                .enableLiveView(false)
//                .setNumDetectorThreads(6)
                .addProcessors(tfod, tfod2)
//                .addProcessor(tfod2)
                .setCamera(hardwareMap[WebcamName::class.java, "Webcam 1"])
                .build()
//                .makeMultiPortalView(2, MultiPortalLayout.HORIZONTAL)
            visionPortal.setProcessorEnabled(tfod, true)
            visionPortal.setProcessorEnabled(tfod2,false)
            telemetry.addLine("Webcam Initiated");
            telemetry.update()
        } else {

            visionPortal = VisionPortal.Builder()
                .enableLiveView(false)
                .addProcessors(tfod, tfod2)
//                .addProcessor(tfod2)
                .setCamera(BuiltinCameraDirection.BACK).build()
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



    private fun telemetryTfod2() {
        val currentRecognitions: List<Recognition> = tfod2.getRecognitions()
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



    /**
     * Read the labels for the object detection model from a file.
     */
    private fun readLabels() {
        val labelList = ArrayList<String>()

        // try to read in the the labels.
        try {
            BufferedReader(FileReader(tfod_labels)).use { br ->
                var index = 0
                while (br.ready()) {
//                    if (index == 0) {
//                        // skip first line.
//                        br.readLine()
//                    } else {
                        labelList.add(br.readLine())
//                    }
                    index++
                }
            }
        } catch (e: Exception) {
            telemetry.addData("Exception", e.localizedMessage)
            telemetry.update()
        }
        if (labelList.size > 0) {
            labels = getStringArray(labelList)
//            RobotLog.vv("readLabels()", "%d labels read.", labels.size)
            telemetry.addData("readLabels()", "%d labels read.", labels.size)

            telemetry.update()
            for (label in labels) {
//                RobotLog.vv("readLabels()", " $label")

                telemetry.addData("readLabels()", " %f", label);

                telemetry.update()
            }
        } else {
//            RobotLog.vv("readLabels()", "No labels read!")
            telemetry.addLine("readLabels() - No Lines read");

            telemetry.update()
        }
    }

    // Function to convert ArrayList<String> to String[]
    private fun getStringArray(arr: List<String>): Array<String> {
        var str = arr.toTypedArray();

        return str
    }


}

