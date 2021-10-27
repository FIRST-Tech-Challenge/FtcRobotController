package com.arcrobotics.ftclib.vision

import org.firstinspires.ftc.robotcore.external.Telemetry
import org.opencv.core.*
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvPipeline

/**
 * This is a Vision Pipeline utilizing Contours and an Aspect Ratio to determine the number of rings
 * currently in the ring stack.
 *
 * @author Team 6133: The "NUTS"
 *
 * Explanation of algorithm:
 * Given an Mat of the current camera view, called input.
 *
 * We first take this Mat and convert it to YCrCb to help with thresholding.
 *
 * We then perform an inRange operation on the input Mat and store the result in a temporary
 * variable called mask. This mask is a black and white image where all white pixels on the mask
 * were pixels in input that are in the orange range threshold. All black pixels on the mask were
 * pixels in the input that were not in the orange range threshold.
 *
 * Next, using a GaussianBlur, we eliminate any noise between the rings on the stack. Due to how we
 * are thresholding in the mask calculation, there may be some gaps in the stack, due to shadows or
 * unwanted light sources.
 *
 * After the GaussianBlur, this noise is eliminated as the picture becomes "blurrier". We then find
 * all contours on the image.
 *
 * What is a Contour? Contours can be explained simply as a curve joining all the continuous points
 * (along the boundary), having same color or intensity. The contours are a useful tool for shape
 * analysis and object detection and recognition.
 *
 * After finding the contours on the black and white mask, we then perform a linear search algorithm
 * on the resulting list of contours (stored in as MatOfPoint). We first find the bounding rectangle
 * of each contour and use this bounding rectangle (not rotated) to find the rectangle with the
 * biggest width. We do this in order to not confuse the ring stack with other objects that may have
 * been thought to be orange by the mask. Since the ring stack will most likely be the largest blob
 * of orange in the view of the camera.
 *
 * When then do a check on the width of the widest contour. To see if it is actually a ring stack,
 * since zero is a valid option we must account for it. This check also makes sure that we don't
 * mistake other smaller objects on the field as the ring stack, even if they are rings.
 *
 * We also implemented a horizon check. Anything above the horizon is disregarded and not looked at
 * even if it has the greatest width from all the other contours. This is to ensure that the
 * algorithm down not detect the red goal as YCrCb color space is very unreliable when detecting the
 * difference between red and orange.
 *
 * After finding the widest contour, which is to be assumed the stack of rings, we perform an aspect
 * ratio of the height over the width of the largest bounding rectangle.
 *
 * Why not just count how tall the largest bounding rectangle is? It is because of camera resolution.
 * Since depending on the resolution of the camera, the height of the stack in pixels would be
 * different despite them both being 4 (lets say for example).
 *
 * Didn't you just say that you used a width check on the contour though? Isn't that also pixels?
 * Yes we did, however, unlike the height of the stack, the width of the stack is consistent. It is
 * always one ring wide, this way we are able to algorithmically generate a minimum bounding width.
 *
 * example of blurring in order to smooth images: https://docs.opencv.org/3.4/dc/dd3/tutorial_gausian_median_blur_bilateral_filter.html
 * example of contours:                           https://docs.opencv.org/3.4/df/d0d/tutorial_find_contours.html
 *
 * @constructor Constructs the VisionPipeline
 *
 * @param telemetry If wanted, provide a telemetry object to the constructor to get stacktrace if an
 * error occurs (mainly for debug purposes)
 * @param debug If true, all intermediate calculation results (except showing mat operations) will
 * be printed to telemetry (mainly for debug purposes)
 */
class UGContourRingPipeline(
        private val telemetry: Telemetry? = null,
        var debug: Boolean = false,
) : OpenCvPipeline() {
    /** variable to store the calculated height of the stack **/
    var height: Height
        private set

    /** variables that will be reused for calculations **/
    private var mat: Mat
    private var ret: Mat

    /** variable to store the rect of the bounding box **/

    private var maxRect = Rect()

    /** variables to store the width, height, and size of the bounding box **/

    val rectWidth
        get() = maxRect.size().width

    val rectHeight
        get() = maxRect.size().height

    val rectSize
        get() = maxRect.size()

    /** enum class for Height of the Ring Stack **/
    enum class Height {
        ZERO, ONE, FOUR
    }

    /** companion object to store all static variables needed **/
    companion object Config {
        /** values used for inRange calculation
         * set to var in-case user wants to use their own tuned values
         * stored in YCrCb format **/
        var lowerOrange = Scalar(0.0, 141.0, 0.0)
        var upperOrange = Scalar(255.0, 230.0, 95.0)

        /** width of the camera in use, defaulted to 320 as that is most common in examples **/
        var CAMERA_WIDTH = 320

        /** Horizon value in use, anything above this value (less than the value) since
         * (0, 0) is the top left of the camera frame **/
        var HORIZON: Int = ((100.0 / 320.0) * CAMERA_WIDTH).toInt()

        /** algorithmically calculated minimum width for width check based on camera width **/
        val MIN_WIDTH
            get() = (50.0 / 320.0) * CAMERA_WIDTH

        /** if the calculated aspect ratio is greater then this, height is 4, otherwise its 1 **/
        const val BOUND_RATIO = 0.7
    }

    /**
     * default init call, body of constructors
     */
    init {
        height = Height.ZERO
        ret = Mat()
        mat = Mat()
    }


    override fun processFrame(input: Mat?): Mat {
        ret.release() // releasing mat to release backing buffer
        // must release at the start of function since this is the variable being returned

        ret = Mat() // resetting pointer held in ret
        try { // try catch in order for opMode to not crash and force a restart
            /**converting from RGB color space to YCrCb color space**/
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2YCrCb)

            /**checking if any pixel is within the orange bounds to make a black and white mask**/
            val mask = Mat(mat.rows(), mat.cols(), CvType.CV_8UC1) // variable to store mask in
            Core.inRange(mat, lowerOrange, upperOrange, mask)

            /**applying to input and putting it on ret in black or yellow**/
            Core.bitwise_and(input, input, ret, mask)

            /**applying GaussianBlur to reduce noise when finding contours**/
            Imgproc.GaussianBlur(mask, mask, Size(5.0, 15.0), 0.00)

            /**finding contours on mask**/
            val contours: List<MatOfPoint> = ArrayList()
            val hierarchy = Mat()
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE)

            /**drawing contours to ret in green**/
            Imgproc.drawContours(ret, contours, -1, Scalar(0.0, 255.0, 0.0), 3)

            /**finding widths of each contour, comparing, and storing the widest**/
            var maxWidth = 0
            for (c: MatOfPoint in contours) {
                val copy = MatOfPoint2f(*c.toArray())
                val rect: Rect = Imgproc.boundingRect(copy)

                val w = rect.width
                // checking if the rectangle is below the horizon
                if (w > maxWidth && rect.y + rect.height > HORIZON) {
                    maxWidth = w
                    maxRect = rect
                }
                c.release() // releasing the buffer of the contour, since after use, it is no longer needed
                copy.release() // releasing the buffer of the copy of the contour, since after use, it is no longer needed
            }

            /**drawing widest bounding rectangle to ret in blue**/
            Imgproc.rectangle(ret, maxRect, Scalar(0.0, 0.0, 255.0), 2)

            /** drawing a red line to show the horizon (any above the horizon is not checked to be a ring stack **/
            Imgproc.line(
                    ret,
                    Point(
                            .0,
                            HORIZON.toDouble()
                    ),
                    Point(
                            CAMERA_WIDTH.toDouble(),
                            HORIZON.toDouble()
                    ),
                    Scalar(
                            255.0,
                            .0,
                            255.0)
            )

            if (debug) telemetry?.addData("Vision: maxW", maxWidth)

            /** checking if widest width is greater than equal to minimum width
             * using Kotlin if expression (Java ternary) to set height variable
             *
             * height = maxWidth >= MIN_WIDTH ? aspectRatio > BOUND_RATIO ? FOUR : ONE : ZERO
             **/
            height = if (maxWidth >= MIN_WIDTH) {
                val aspectRatio: Double = maxRect.height.toDouble() / maxRect.width.toDouble()

                if (debug) telemetry?.addData("Vision: Aspect Ratio", aspectRatio)

                /** checks if aspectRatio is greater than BOUND_RATIO
                 * to determine whether stack is ONE or FOUR
                 */
                if (aspectRatio > BOUND_RATIO)
                    Height.FOUR // height variable is now FOUR
                else
                    Height.ONE // height variable is now ONE
            } else {
                Height.ZERO // height variable is now ZERO
            }

            if (debug) telemetry?.addData("Vision: Height", height)

            // releasing all mats after use
            mat.release()
            mask.release()
            hierarchy.release()

        } catch (e: Exception) {
            /**error handling, prints stack trace for specific debug**/
            telemetry?.addData("[ERROR]", e)
            e.stackTrace.toList().stream().forEach { x -> telemetry?.addLine(x.toString()) }
        }
        telemetry?.update()

        /**returns the black and orange mask with contours drawn to see logic in action**/
        return ret
    }
}