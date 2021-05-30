package org.firstinspires.ftc.teamcode.imgproc

import android.graphics.Bitmap
import android.graphics.RectF
import android.util.Log
import androidx.appcompat.app.AppCompatActivity
import com.qualcomm.hardware.ams.AMSColorSensor
import kotlinx.coroutines.*
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap
import org.firstinspires.ftc.robotcore.external.ClassFactory
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector
import org.firstinspires.ftc.teamcode.R
import org.tensorflow.lite.support.image.TensorImage
import org.tensorflow.lite.task.vision.detector.Detection
import org.tensorflow.lite.task.vision.detector.ObjectDetector

class ImgProc() : AppCompatActivity() {

    private val options = ObjectDetector.ObjectDetectorOptions.builder()
            .setMaxResults(5)
            .setScoreThreshold(0.5f)
            .build()

    val objectDetector: ObjectDetector = ObjectDetector.createFromFileAndOptions(
            this,
            "UltimateGoal.tflite",
            options,
    )

    private val VUFORIA_KEY = resources.getString(R.string.VUFORIA_KEY)
    private val CAM_NAME = resources.getString(R.string.webcam)

    init {
        initVuforia()
        initTF()
    }

    lateinit var vuforia: VuforiaLocalizer
    lateinit var tfod: TFObjectDetector

    private fun initVuforia() {
        val parameters = VuforiaLocalizer.Parameters()

        parameters.vuforiaLicenseKey = VUFORIA_KEY
        parameters.cameraName = hardwareMap[WebcamName::class.java, CAM_NAME]

        //  Instantiate the Vuforia engine
        this.vuforia = ClassFactory.getInstance().createVuforia(parameters)
    }

    private fun initTF() {
        val tfodMonitorViewId = hardwareMap.appContext.resources.getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.packageName)

        val tfodParameters = TFObjectDetector.Parameters(tfodMonitorViewId)

        tfodParameters.minResultConfidence = 0.66f

        this.tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia)

        this.tfod.setZoom(2.5, 16.0 / 9.0)
    }

    private fun debugPrint(results: List<Detection>) {
        val TAG = "ImageProc"
        for ((i, obj) in results.withIndex()) {
            val box = obj.boundingBox

            Log.d(TAG, "Detected object: $i ")
            Log.d(TAG, "  boundingBox: (${box.left}, ${box.top}) - (${box.right},${box.bottom})")

            for ((j, category) in obj.categories.withIndex()) {
                Log.d(TAG, "    Label $j: ${category.label}")
                val confidence: Int = category.score.times(100).toInt()
                Log.d(TAG, "    Confidence: ${confidence}%")
            }
        }
    }

    lateinit var objects: MutableList<ObjectDetected>

    private suspend fun loop(telemetry: Telemetry) {
        while (true) {
            Thread.sleep(10)
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                val updatedRecognitions = tfod.updatedRecognitions
                if (updatedRecognitions != null) {
                    this.objects.clear()
                    telemetry.addData("# Object Detected", updatedRecognitions.size)
                    // step through the list of recognitions and display boundary info.
                    val i = 0
                    for (recognition in updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.label)
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.left, recognition.top)
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.right, recognition.bottom)

                        this.objects.add(ObjectDetected(
                                recognition.label,
                                RectF(
                                        recognition.left,
                                        recognition.top,
                                        recognition.right,
                                        recognition.bottom,
                                ),
                        ))
                    }
                    telemetry.update()
                }
            }
        }
    }

    lateinit var looper: Deferred<Unit>

    fun start(telemetry: Telemetry) {
        tfod.activate()

        this.looper = GlobalScope.async(Dispatchers.Default) {
            loop(telemetry)
        }
    }

    fun stop() {
        tfod.deactivate()

        this.looper.cancel()
    }

    fun processLegacy(bitmap: Bitmap): Array<ObjectDetected> {
        val image = TensorImage.fromBitmap(bitmap)

        val results = this.objectDetector.detect(image)

        debugPrint(results)

        val objects: MutableList<ObjectDetected> = ArrayList()

        for (obj in results) {
            val box = obj.boundingBox
            val category = obj.categories.first()
            val type = category.label.toString()

            objects.add(ObjectDetected(type, box))
        }

        return objects.toTypedArray()
    }

    fun retrieve(): Array<ObjectDetected> {
        return this.objects.toTypedArray()
    }
}

data class ObjectDetected(val label: String, val boundingBox: RectF)