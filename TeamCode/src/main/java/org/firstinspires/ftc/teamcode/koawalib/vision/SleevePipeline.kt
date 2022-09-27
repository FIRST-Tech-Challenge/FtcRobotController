/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
package org.firstinspires.ftc.teamcode.koawalib.vision

import org.opencv.calib3d.Calib3d
import org.opencv.core.*
import org.opencv.imgproc.Imgproc
import org.openftc.apriltag.AprilTagDetection
import org.openftc.apriltag.AprilTagDetectorJNI
import org.openftc.easyopencv.OpenCvPipeline

class SleevePipeline(// UNITS ARE METERS
    var tagsize: Double, fx: Double, fy: Double, cx: Double, cy: Double
) :
    OpenCvPipeline() {
    private var nativeApriltagPtr: Long
    private val grey = Mat()
    var latestDetections = ArrayList<AprilTagDetection>()
        private set
    private var detectionsUpdate: ArrayList<AprilTagDetection> = ArrayList()
    private val detectionsUpdateSync = Any()
    var cameraMatrix = Mat()
    var blue = Scalar(7.0, 197.0, 235.0, 255.0)
    var red = Scalar(255.0, 0.0, 0.0, 255.0)
    var green = Scalar(0.0, 255.0, 0.0, 255.0)
    var white = Scalar(255.0, 255.0, 255.0, 255.0)
    var fx: Double
    var fy: Double
    var cx: Double
    var cy: Double
    var tagsizeX: Double
    var tagsizeY: Double
    private var decimation = 0f
    private var needToSetDecimation = false
    private val decimationSync = Any()

    init {
        tagsizeX = tagsize
        tagsizeY = tagsize
        this.fx = fx
        this.fy = fy
        this.cx = cx
        this.cy = cy
        constructMatrix()

        // Allocate a native context object. See the corresponding deletion in the finalizer
        nativeApriltagPtr = AprilTagDetectorJNI.createApriltagDetector(
            AprilTagDetectorJNI.TagFamily.TAG_36h11.string,
            3f,
            3
        )
    }

    fun finalize() {
        // Might be null if createApriltagDetector() threw an exception
        if (nativeApriltagPtr != 0L) {
            // Delete the native context we created in the constructor
            AprilTagDetectorJNI.releaseApriltagDetector(nativeApriltagPtr)
            nativeApriltagPtr = 0
        } else {
            println("AprilTagDetectionPipeline.finalize(): nativeApriltagPtr was NULL")
        }
    }

    override fun processFrame(input: Mat): Mat {
        // Convert to greyscale
        Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGBA2GRAY)
        synchronized(decimationSync) {
            if (needToSetDecimation) {
                AprilTagDetectorJNI.setApriltagDetectorDecimation(nativeApriltagPtr, decimation)
                needToSetDecimation = false
            }
        }

        // Run AprilTag
        latestDetections = AprilTagDetectorJNI.runAprilTagDetectorSimple(
            nativeApriltagPtr, grey,
            tagsize, fx, fy, cx, cy
        )
        synchronized(detectionsUpdateSync) { detectionsUpdate = latestDetections }

        // For fun, use OpenCV to draw 6DOF markers on the image. We actually recompute the pose using
        // OpenCV because I haven't yet figured out how to re-use AprilTag's pose in OpenCV.
        for (detection in latestDetections) {
            val pose = poseFromTrapezoid(detection.corners, cameraMatrix, tagsizeX, tagsizeY)
            drawAxisMarker(input, tagsizeY / 2.0, 6, pose.rvec, pose.tvec, cameraMatrix)
            draw3dCubeMarker(
                input,
                tagsizeX,
                tagsizeX,
                tagsizeY,
                5,
                pose.rvec,
                pose.tvec,
                cameraMatrix
            )
        }
        return input
    }

    fun setDecimation(decimation: Float) {
        synchronized(decimationSync) {
            this.decimation = decimation
            needToSetDecimation = true
        }
    }

    fun getDetectionsUpdate(): ArrayList<AprilTagDetection> {
        synchronized(detectionsUpdateSync) {
            val ret = detectionsUpdate
            detectionsUpdate
            return ret
        }
    }

    fun constructMatrix() {
        //     Construct the camera matrix.
        //
        //      --         --
        //     | fx   0   cx |
        //     | 0    fy  cy |
        //     | 0    0   1  |
        //      --         --
        //
        cameraMatrix = Mat(3, 3, CvType.CV_32FC1)
        cameraMatrix.put(0, 0, fx)
        cameraMatrix.put(0, 1, 0.0)
        cameraMatrix.put(0, 2, cx)
        cameraMatrix.put(1, 0, 0.0)
        cameraMatrix.put(1, 1, fy)
        cameraMatrix.put(1, 2, cy)
        cameraMatrix.put(2, 0, 0.0)
        cameraMatrix.put(2, 1, 0.0)
        cameraMatrix.put(2, 2, 1.0)
    }

    /**
     * Draw a 3D axis marker on a detection. (Similar to what Vuforia does)
     *
     * @param buf the RGB buffer on which to draw the marker
     * @param length the length of each of the marker 'poles'
     * @param rvec the rotation vector of the detection
     * @param tvec the translation vector of the detection
     * @param cameraMatrix the camera matrix used when finding the detection
     */
    fun drawAxisMarker(
        buf: Mat,
        length: Double,
        thickness: Int,
        rvec: Mat,
        tvec: Mat,
        cameraMatrix: Mat
    ) {
        // The points in 3D space we wish to project onto the 2D image plane.
        // The origin of the coordinate space is assumed to be in the center of the detection.
        val axis = MatOfPoint3f(
            Point3(0.0, 0.0, 0.0),
            Point3(length, 0.0, 0.0),
            Point3(0.0, length, 0.0),
            Point3(0.0, 0.0, -length)
        )

        // Project those points
        val matProjectedPoints = MatOfPoint2f()
        Calib3d.projectPoints(axis, rvec, tvec, cameraMatrix, MatOfDouble(), matProjectedPoints)
        val projectedPoints = matProjectedPoints.toArray()

        // Draw the marker!
        Imgproc.line(buf, projectedPoints[0], projectedPoints[1], red, thickness)
        Imgproc.line(buf, projectedPoints[0], projectedPoints[2], green, thickness)
        Imgproc.line(buf, projectedPoints[0], projectedPoints[3], blue, thickness)
        Imgproc.circle(buf, projectedPoints[0], thickness, white, -1)
    }

    fun draw3dCubeMarker(
        buf: Mat,
        length: Double,
        tagWidth: Double,
        tagHeight: Double,
        thickness: Int,
        rvec: Mat,
        tvec: Mat,
        cameraMatrix: Mat
    ) {
        //axis = np.float32([[0,0,0], [0,3,0], [3,3,0], [3,0,0],
        //       [0,0,-3],[0,3,-3],[3,3,-3],[3,0,-3] ])

        // The points in 3D space we wish to project onto the 2D image plane.
        // The origin of the coordinate space is assumed to be in the center of the detection.
        val axis = MatOfPoint3f(
            Point3(-tagWidth / 2, tagHeight / 2, 0.0),
            Point3(tagWidth / 2, tagHeight / 2, 0.0),
            Point3(tagWidth / 2, -tagHeight / 2, 0.0),
            Point3(-tagWidth / 2, -tagHeight / 2, 0.0),
            Point3(-tagWidth / 2, tagHeight / 2, -length),
            Point3(tagWidth / 2, tagHeight / 2, -length),
            Point3(tagWidth / 2, -tagHeight / 2, -length),
            Point3(-tagWidth / 2, -tagHeight / 2, -length)
        )

        // Project those points
        val matProjectedPoints = MatOfPoint2f()
        Calib3d.projectPoints(axis, rvec, tvec, cameraMatrix, MatOfDouble(), matProjectedPoints)
        val projectedPoints = matProjectedPoints.toArray()

        // Pillars
        for (i in 0..3) {
            Imgproc.line(buf, projectedPoints[i], projectedPoints[i + 4], blue, thickness)
        }

        // Base lines
        //Imgproc.line(buf, projectedPoints[0], projectedPoints[1], blue, thickness);
        //Imgproc.line(buf, projectedPoints[1], projectedPoints[2], blue, thickness);
        //Imgproc.line(buf, projectedPoints[2], projectedPoints[3], blue, thickness);
        //Imgproc.line(buf, projectedPoints[3], projectedPoints[0], blue, thickness);

        // Top lines
        Imgproc.line(buf, projectedPoints[4], projectedPoints[5], green, thickness)
        Imgproc.line(buf, projectedPoints[5], projectedPoints[6], green, thickness)
        Imgproc.line(buf, projectedPoints[6], projectedPoints[7], green, thickness)
        Imgproc.line(buf, projectedPoints[4], projectedPoints[7], green, thickness)
    }

    /**
     * Extracts 6DOF pose from a trapezoid, using a camera intrinsics matrix and the
     * original size of the tag.
     *
     * @param points the points which form the trapezoid
     * @param cameraMatrix the camera intrinsics matrix
     * @param tagsizeX the original width of the tag
     * @param tagsizeY the original height of the tag
     * @return the 6DOF pose of the camera relative to the tag
     */
    fun poseFromTrapezoid(
        points: Array<Point>,
        cameraMatrix: Mat,
        tagsizeX: Double,
        tagsizeY: Double
    ): Pose {
        // The actual 2d points of the tag detected in the image
        val points2d = MatOfPoint2f(*points)

        // The 3d points of the tag in an 'ideal projection'
        val arrayPoints3d = arrayOfNulls<Point3>(4)
        arrayPoints3d[0] = Point3(-tagsizeX / 2, tagsizeY / 2, 0.0)
        arrayPoints3d[1] = Point3(tagsizeX / 2, tagsizeY / 2, 0.0)
        arrayPoints3d[2] = Point3(tagsizeX / 2, -tagsizeY / 2, 0.0)
        arrayPoints3d[3] = Point3(-tagsizeX / 2, -tagsizeY / 2, 0.0)
        val points3d = MatOfPoint3f(*arrayPoints3d)

        // Using this information, actually solve for pose
        val pose: Pose = Pose()
        Calib3d.solvePnP(
            points3d,
            points2d,
            cameraMatrix,
            MatOfDouble(),
            pose.rvec,
            pose.tvec,
            false
        )
        return pose
    }

    /*
     * A simple container to hold both rotation and translation
     * vectors, which together form a 6DOF pose.
     */
    inner class Pose {
        var rvec: Mat
        var tvec: Mat

        constructor() {
            rvec = Mat()
            tvec = Mat()
        }

        constructor(rvec: Mat, tvec: Mat) {
            this.rvec = rvec
            this.tvec = tvec
        }
    }
}