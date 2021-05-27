package org.firstinspires.ftc.teamcode.imgproc

import android.graphics.Bitmap
import android.graphics.RectF
import android.os.Bundle
import android.util.Log
import androidx.appcompat.app.AppCompatActivity
import org.tensorflow.lite.support.image.TensorImage
import org.tensorflow.lite.task.vision.detector.Detection
import org.tensorflow.lite.task.vision.detector.ObjectDetector

class ImgProc : AppCompatActivity() {

    private val options = ObjectDetector.ObjectDetectorOptions.builder()
            .setMaxResults(5)
            .setScoreThreshold(0.5f)
            .build()

    val objectDetector = ObjectDetector.createFromFileAndOptions(
            this,
            "UltimateGoal.tflite",
            options,
    )

    fun debugPrint(results : List<Detection>) {
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

    fun process(bitmap: Bitmap): Array<Object> {
        val image = TensorImage.fromBitmap(bitmap)

        val results = this.objectDetector.detect(image)

        debugPrint(results)

        val objects: MutableList<Object> = ArrayList()

        for (obj in results) {
            val box = obj.boundingBox
            val category = obj.categories.first()
            val type = category.label.toString()

            objects.add(Object(type, box))
        }

        return objects.toTypedArray()
    }
}

data class Object(val label: String, val boundingBox: RectF)