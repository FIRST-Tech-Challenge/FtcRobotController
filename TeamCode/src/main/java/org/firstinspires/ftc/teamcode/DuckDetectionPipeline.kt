package org.firstinspires.ftc.teamcode

import org.opencv.core.*
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvPipeline

class DuckDetectionPipeline : OpenCvPipeline() {
    // HSV Color bounds for inRange() check
    val lowerBound = Scalar(10.0, 200.0, 150.0)
    val upperBound = Scalar(20.0, 255.0, 255.0)

    // Mats defined in order of how they are used in processFrame()
    var hsv = Mat() // inputMat converted from RGB to HSV
    var mask = Mat() // hsv with isolated parts of interest color
    val kernel =  Imgproc.getStructuringElement(Imgproc.MORPH_RECT, Size(3.0, 3.0))
    var cleanMask = Mat() // same as mask, but eroded and dilated to clean up stray pixels
    var result = Mat() // The original image with boxes around color areas
    var hierarchy = Mat()

    val contours = ArrayList<MatOfPoint>()

    override fun processFrame(input: Mat?): Mat {
        contours.clear()

        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV)
        Core.inRange(hsv, lowerBound, upperBound, mask)
        Imgproc.morphologyEx(mask, cleanMask, Imgproc.MORPH_OPEN, kernel)
        Imgproc.findContours(cleanMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE)

        result.setTo(input)
        for (c in contours) {
            if (Imgproc.contourArea(c) < 50) continue

            val boundingRect = Imgproc.boundingRect(c)
            val pt1 = Point(boundingRect.x.toDouble(), boundingRect.y.toDouble())
            val pt2 = Point((boundingRect.x + boundingRect.width).toDouble(),  (boundingRect.y + boundingRect.height).toDouble())
            Imgproc.rectangle(result, pt1, pt2, Scalar(0.0, 0.0, 255.0))
        }
        Imgproc.cvtColor(result, result, Imgproc.COLOR_HSV2RGB)
        return result
    }
}