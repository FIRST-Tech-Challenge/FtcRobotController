package teamcode.v1.vision

import org.opencv.calib3d.Calib3d
import org.opencv.core.*
import org.opencv.imgproc.Imgproc
import org.openftc.apriltag.AprilTagDetection
import org.openftc.apriltag.AprilTagDetectorJNI
import org.openftc.easyopencv.OpenCvPipeline

class SleevePipeline : OpenCvPipeline() {
    data class PipelinePose(val rvec: Mat = Mat(), val tvec: Mat = Mat())

    private var nativeApriltagPtr: Long
    private val grey = Mat()
    private var latestDetections = ArrayList<AprilTagDetection>()
    private var detectionsUpdate: ArrayList<AprilTagDetection> = ArrayList()
    private val detectionsUpdateSync = Any()
    private var cameraMatrix = Mat()
    private val blue = Scalar(7.0, 197.0, 235.0, 255.0)
    private val red = Scalar(255.0, 0.0, 0.0, 255.0)
    private val green = Scalar(0.0, 255.0, 0.0, 255.0)
    private val white = Scalar(255.0, 255.0, 255.0, 255.0)
    private val zoneList = listOf(LEFT, MIDDLE, RIGHT)

    var zone = Enums.Zones.WTF
        private set

    private fun drawAxisMarker(
        buf: Mat,
        rvec: Mat,
        tvec: Mat,
        cameraMatrix: Mat
    ) {
        val length = tagsize / 2.0
        val thickness = 6
        val axis = MatOfPoint3f(
            Point3(0.0, 0.0, 0.0),
            Point3(length, 0.0, 0.0),
            Point3(0.0, length, 0.0),
            Point3(0.0, 0.0, -length)
        )

        val matProjectedPoints = MatOfPoint2f()
        Calib3d.projectPoints(axis, rvec, tvec, cameraMatrix, MatOfDouble(), matProjectedPoints)
        val projectedPoints = matProjectedPoints.toArray()

        // Draw the marker!
        Imgproc.line(buf, projectedPoints[0], projectedPoints[1], red, thickness)
        Imgproc.line(buf, projectedPoints[0], projectedPoints[2], green, thickness)
        Imgproc.line(buf, projectedPoints[0], projectedPoints[3], blue, thickness)
        Imgproc.circle(buf, projectedPoints[0], thickness, white, -1)
    }

    private fun draw3dCubeMarker(
        buf: Mat,
        rvec: Mat,
        tvec: Mat,
        cameraMatrix: Mat
    ) {
        val thickness = 5
        val axis = MatOfPoint3f(
            Point3(-tagsize / 2, tagsize / 2, 0.0),
            Point3(tagsize / 2, tagsize / 2, 0.0),
            Point3(tagsize / 2, -tagsize / 2, 0.0),
            Point3(-tagsize / 2, -tagsize / 2, 0.0),
            Point3(-tagsize / 2, tagsize / 2, -tagsize),
            Point3(tagsize / 2, tagsize / 2, -tagsize),
            Point3(tagsize / 2, -tagsize / 2, -tagsize),
            Point3(-tagsize / 2, -tagsize / 2, -tagsize)
        )

        val matProjectedPoints = MatOfPoint2f()
        Calib3d.projectPoints(axis, rvec, tvec, cameraMatrix, MatOfDouble(), matProjectedPoints)
        val projectedPoints = matProjectedPoints.toArray()

        for (i in 0..3) {
            Imgproc.line(buf, projectedPoints[i], projectedPoints[i + 4], blue, thickness)
        }

        Imgproc.line(buf, projectedPoints[4], projectedPoints[5], green, thickness)
        Imgproc.line(buf, projectedPoints[5], projectedPoints[6], green, thickness)
        Imgproc.line(buf, projectedPoints[6], projectedPoints[7], green, thickness)
        Imgproc.line(buf, projectedPoints[4], projectedPoints[7], green, thickness)
    }

    private fun poseFromTrapezoid(
        points: Array<Point>,
        cameraMatrix: Mat,
    ): PipelinePose {
        val points2d = MatOfPoint2f(*points)

        val arrayPoints3d = arrayOfNulls<Point3>(4)
        arrayPoints3d[0] = Point3(-tagsize / 2, tagsize / 2, 0.0)
        arrayPoints3d[1] = Point3(tagsize / 2, tagsize / 2, 0.0)
        arrayPoints3d[2] = Point3(tagsize / 2, -tagsize / 2, 0.0)
        arrayPoints3d[3] = Point3(-tagsize / 2, -tagsize / 2, 0.0)
        val points3d = MatOfPoint3f(*arrayPoints3d)

        val pose = PipelinePose()
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

    override fun processFrame(input: Mat): Mat {
        Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGBA2GRAY)
        latestDetections = AprilTagDetectorJNI.runAprilTagDetectorSimple(
            nativeApriltagPtr, grey,
            tagsize, fx, fy, cx, cy
        )
        synchronized(detectionsUpdateSync) { detectionsUpdate = latestDetections }

        for (detection in latestDetections) {
            val pose = poseFromTrapezoid(detection.corners, cameraMatrix)
            drawAxisMarker(input, pose.rvec, pose.tvec, cameraMatrix)
            draw3dCubeMarker(
                input,
                pose.rvec,
                pose.tvec,
                cameraMatrix
            )

            if(detection.id in zoneList) {
                zone = when(detection.id) {
                    1 -> Enums.Zones.LEFT
                    2 -> Enums.Zones.MIDDLE
                    3 -> Enums.Zones.RIGHT
                    else -> Enums.Zones.WTF
                }
            }
        }
        return input
    }

    fun release() {
        if (nativeApriltagPtr != 0L) {
            AprilTagDetectorJNI.releaseApriltagDetector(nativeApriltagPtr)
            nativeApriltagPtr = 0
        } else {
            println("SleevePipeline.finalize(): nativeApriltagPtr was NULL")
        }
        grey.release()
        cameraMatrix.release()
    }

    init {
        val data = listOf(fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0)
        cameraMatrix = Mat(3, 3, CvType.CV_32FC1)
        for(r in 0 until  cameraMatrix.rows()) {
            for(c in 0 until cameraMatrix.cols()) {
                cameraMatrix.put(r, c, data[r * 3 + c])
            }
        }

        nativeApriltagPtr = AprilTagDetectorJNI.createApriltagDetector(
            AprilTagDetectorJNI.TagFamily.TAG_36h11.string,
            3f,
            3
        )
    }

    companion object {
        private const val fx = 578.272
        private const val fy = 578.272
        private const val cx = 402.145
        private const val cy = 221.506
        private const val tagsize = 0.166
        private const val LEFT = 1
        private const val MIDDLE = 2
        private const val RIGHT = 3
    }
}