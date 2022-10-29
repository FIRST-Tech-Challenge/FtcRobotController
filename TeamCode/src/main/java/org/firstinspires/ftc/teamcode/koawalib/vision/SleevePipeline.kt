package org.firstinspires.ftc.teamcode.koawalib.vision

import org.opencv.core.*
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvPipeline

class SleevePipeline : OpenCvPipeline() {
    /*
    YELLOW  = Parking Left
    CYAN    = Parking Middle
    MAGENTA = Parking Right
     */
    enum class ParkingPosition {
        LEFT, CENTER, RIGHT
    }

    // Color definitions
    private val YELLOW = Scalar(255.0, 255.0, 0.0)
    private val CYAN = Scalar(0.0, 255.0, 255.0)
    private val MAGENTA = Scalar(255.0, 0.0, 255.0)

    // Percent and mat definitions
    private var yelPercent = 0.0
    private var cyaPercent = 0.0
    private var magPercent = 0.0
    private val yelMat = Mat()
    private val cyaMat = Mat()
    private val magMat = Mat()
    private var blurredMat = Mat()
    private var kernel = Mat()

    // Anchor point definitions
    var sleeve_pointA = Point(
        SLEEVE_TOPLEFT_ANCHOR_POINT.x,
        SLEEVE_TOPLEFT_ANCHOR_POINT.y
    )
    var sleeve_pointB = Point(
        SLEEVE_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
        SLEEVE_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT
    )

    // Returns an enum being the current position where the robot will park
    // Running variable storing the parking position
    @Volatile
    var position = ParkingPosition.LEFT
        private set

    override fun processFrame(input: Mat): Mat {
        // Noise reduction
        Imgproc.blur(input, blurredMat, Size(5.0, 5.0))
        blurredMat = blurredMat.submat(Rect(sleeve_pointA, sleeve_pointB))

        // Apply Morphology
        kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, Size(3.0, 3.0))
        Imgproc.morphologyEx(blurredMat, blurredMat, Imgproc.MORPH_CLOSE, kernel)

        // Gets channels from given source mat
        Core.inRange(blurredMat, lower_yellow_bounds, upper_yellow_bounds, yelMat)
        Core.inRange(blurredMat, lower_cyan_bounds, upper_cyan_bounds, cyaMat)
        Core.inRange(blurredMat, lower_magenta_bounds, upper_magenta_bounds, magMat)

        // Gets color specific values
        yelPercent = Core.countNonZero(yelMat).toDouble()
        cyaPercent = Core.countNonZero(cyaMat).toDouble()
        magPercent = Core.countNonZero(magMat).toDouble()

        // Calculates the highest amount of pixels being covered on each side
        val maxPercent = Math.max(yelPercent, Math.max(cyaPercent, magPercent))

        // Checks all percentages, will highlight bounding box in camera preview
        // based on what color is being detected
        if (maxPercent == yelPercent) {
            position = ParkingPosition.LEFT
            Imgproc.rectangle(
                input,
                sleeve_pointA,
                sleeve_pointB,
                YELLOW,
                2
            )
        } else if (maxPercent == cyaPercent) {
            position = ParkingPosition.CENTER
            Imgproc.rectangle(
                input,
                sleeve_pointA,
                sleeve_pointB,
                CYAN,
                2
            )
        } else if (maxPercent == magPercent) {
            position = ParkingPosition.RIGHT
            Imgproc.rectangle(
                input,
                sleeve_pointA,
                sleeve_pointB,
                MAGENTA,
                2
            )
        }

        // Memory cleanup
        blurredMat.release()
        yelMat.release()
        cyaMat.release()
        magMat.release()
        kernel.release()
        return input
    }

    companion object {
        // TOPLEFT anchor point for the bounding box
        private val SLEEVE_TOPLEFT_ANCHOR_POINT = Point(145.0, 168.0)

        // Width and height for the bounding box
        var REGION_WIDTH = 30
        var REGION_HEIGHT = 50

        // Lower and upper boundaries for colors
        private val lower_yellow_bounds = Scalar(200.0, 200.0, 0.0, 255.0)
        private val upper_yellow_bounds = Scalar(255.0, 255.0, 130.0, 255.0)
        private val lower_cyan_bounds = Scalar(0.0, 200.0, 200.0, 255.0)
        private val upper_cyan_bounds = Scalar(150.0, 255.0, 255.0, 255.0)
        private val lower_magenta_bounds = Scalar(170.0, 0.0, 170.0, 255.0)
        private val upper_magenta_bounds = Scalar(255.0, 60.0, 255.0, 255.0)
    }
}