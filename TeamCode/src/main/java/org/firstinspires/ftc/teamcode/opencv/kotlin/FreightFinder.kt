package org.firstinspires.ftc.teamcode.opencv.kotlin

import org.opencv.core.*
import org.openftc.easyopencv.OpenCvPipeline
import org.opencv.imgproc.Imgproc
import java.util.ArrayList

class FreightFinder : OpenCvPipeline() {
    var boxArray: ArrayList<Rect> = ArrayList()
    var ballArray: ArrayList<Rect> = ArrayList()

    val whiteContours: List<MatOfPoint> = ArrayList()
    val yellowContours: List<MatOfPoint> = ArrayList()

    override fun processFrame(input: Mat): Mat {
        val white = Mat()
        val orange = Mat()

        val lowWhite = Scalar(0.0, 0.0, 200.0) // wiffleball lower
        val highWhite = Scalar(180.0, 10.0, 255.0) // wiffleball upper
        val lowOrange = Scalar(14.0, 130.0, 80.0) // block lower
        val highOrange = Scalar(25.0, 255.0, 255.0) // block upper

        Imgproc.cvtColor(input, white, Imgproc.COLOR_RGB2HSV)
        Imgproc.cvtColor(input, orange, Imgproc.COLOR_RGB2HSV)

        Core.inRange(white, lowWhite, highWhite, white)
        Core.inRange(orange, lowOrange, highOrange, orange)


        Imgproc.GaussianBlur(white, white, Size(5.0, 5.0), 0.0, 0.0)
        Imgproc.GaussianBlur(orange, orange, Size(5.0, 5.0), 0.0, 0.0)



        val hierarchy1 = Mat()
        val hierarchy2 = Mat()

        Imgproc.findContours(white, whiteContours, hierarchy1, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE)
        Imgproc.findContours(orange, yellowContours, hierarchy2, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE)

        Imgproc.drawContours(input, whiteContours, -1, Scalar(0.0, 255.0, 0.0), 3)
        Imgproc.drawContours(input, yellowContours, -1, Scalar(0.0, 0.0, 255.0), 3)

        for (contour in whiteContours) {
            val rect = Imgproc.boundingRect(contour)
            ballArray.add(rect)
        }
        for (contour in yellowContours) {
            val rect = Imgproc.boundingRect(contour)
            boxArray.add(rect)
        }


        return input
    }
}