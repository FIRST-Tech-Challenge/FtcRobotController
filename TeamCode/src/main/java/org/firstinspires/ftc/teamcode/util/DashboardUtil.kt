package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.path.Path
import kotlin.math.ceil

/**
 * Set of helper functions for drawing Road Runner paths and trajectories on dashboard canvases.
 */
object DashboardUtil {
    private const val DEFAULT_RESOLUTION = 2.0 // distance units; presumed inches
    private const val ROBOT_RADIUS = 9.0 // in


    fun drawPoseHistory(canvas: Canvas, poseHistory: List<Pose2d>) {
        val xPoints = DoubleArray(poseHistory.size)
        val yPoints = DoubleArray(poseHistory.size)
        for (i in poseHistory.indices) {
            val pose: Pose2d = poseHistory[i]
            xPoints[i] = pose.x
            yPoints[i] = pose.y
        }
        canvas.strokePolyline(xPoints, yPoints)
    }

    @JvmOverloads
    fun drawSampledPath(canvas: Canvas, path: Path, resolution: Double = DEFAULT_RESOLUTION) {
        val samples = ceil(path.length() / resolution).toInt()
        val xPoints = DoubleArray(samples)
        val yPoints = DoubleArray(samples)
        val dx: Double = path.length() / (samples - 1)
        for (i in 0 until samples) {
            val displacement: Double = i * dx
            val pose: Pose2d = path[displacement]
            xPoints[i] = pose.x
            yPoints[i] = pose.y
        }
        canvas.strokePolyline(xPoints, yPoints)
    }

    fun drawRobot(canvas: Canvas, pose: Pose2d) {
        canvas.strokeCircle(pose.x, pose.y, ROBOT_RADIUS)
        val v: Vector2d = pose.headingVec().times(ROBOT_RADIUS)
        val x1: Double = pose.x + v.x / 2
        val y1: Double = pose.y + v.y / 2
        val x2: Double = pose.x + v.x
        val y2: Double = pose.y + v.y
        canvas.strokeLine(x1, y1, x2, y2)
    }
}