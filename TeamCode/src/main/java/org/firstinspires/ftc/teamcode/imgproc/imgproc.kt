package org.firstinspires.ftc.teamcode.imgproc

import android.graphics.Bitmap
import android.os.Bundle
import android.util.Log
import androidx.appcompat.app.AppCompatActivity
import org.tensorflow.lite.support.image.TensorImage
import org.tensorflow.lite.task.vision.detector.Detection
import org.tensorflow.lite.task.vision.detector.ObjectDetector

class ImgProc : AppCompatActivity() {

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
    }

    fun debugPrint(results : List<Detection>) {
        val TAG = "ImageProc"
        for ((i, obj) in results.withIndex()) {
            val box = obj.boundingBox

            Log.d(TAG, "Detected object: ${i} ")
            Log.d(TAG, "  boundingBox: (${box.left}, ${box.top}) - (${box.right},${box.bottom})")

            for ((j, category) in obj.categories.withIndex()) {
                Log.d(TAG, "    Label $j: ${category.label}")
                val confidence: Int = category.score.times(100).toInt()
                Log.d(TAG, "    Confidence: ${confidence}%")
            }
        }
    }

    fun process(bitmap: Bitmap): MutableList<Array<String>> {
        val image = TensorImage.fromBitmap(bitmap)

        val options = ObjectDetector.ObjectDetectorOptions.builder()
                .setMaxResults(5)
                .setScoreThreshold(0.5f)
                .build()

        val detector = ObjectDetector.createFromFileAndOptions(
                this,
                "UltimateGoal.tflite",
                options,
        )

        val results = detector.detect(image)

        debugPrint(results)

        val objects: MutableList<Array<String>> = ArrayList()

        for (obj in results) {
            val box = obj.boundingBox
            val category = obj.categories.first()
            val type = category.label.toString()
            val add = [type, box.left.toString(), box.top.toString(), box.right.toString(), box.left.toString()]
            objects.add(add)
        }

        return objects
    }
}