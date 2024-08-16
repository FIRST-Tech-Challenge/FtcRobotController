package com.millburnx.purePursuit

import com.millburnx.purePursuit.Utils.Intersection
import com.millburnx.purePursuit.Utils.LineSegment
import com.millburnx.purePursuit.Utils.Point
import com.millburnx.purePursuit.Utils.Utils
import com.millburnx.purePursuit.ftcDashboard.ICanvas
import com.millburnx.purePursuit.ftcDashboard.TelemetryPacket
import kotlin.math.abs

class PurePursuit(ppi: Double, updateHertz: Double = -1.0) : OpMode(ppi, updateHertz) {
    val path: List<Point> = listOf(
        Point(0.0, 0.0),
        Point(48.0, 0.0),
        Point(48.0, -48.0),
        Point(0.0, -48.0)
    )
    val pathSegments: List<LineSegment> = path.zipWithNext().map { LineSegment(it.first, it.second) }

    var lastIntersection: Intersection = Intersection(path[0], pathSegments[0]) // start of the path
    var lastSegment: Int = 0 // prevent backtracking

    val prevPosition: MutableList<Point> = mutableListOf()

    override fun init() {
        println("Initializing Pure Pursuit")
    }

    override fun loop(): Boolean {
        val packet = TelemetryPacket()
        val canvas = packet.fieldOverlay()
            .drawImage("PathPlanner/src/main/resources/bg.png", 0.0, 0.0, 144.0, 144.0)
            .setFill("#808080")
            .drawGrid(0.0, 0.0, 144.0, 144.0, 7, 7)
            .setFill("#D3D3D3")
        for (point in path) {
            canvas.fillCircle(point.x, point.y, 1.0)
        }
        canvas.setStrokeWidth(2)
        for (segment in pathSegments) {
            canvas.strokeLine(segment.p1.x, segment.p1.y, segment.p2.x, segment.p2.y)
        }

        val distanceToFinal = robot.position.distanceTo(path.last())
        if (distanceToFinal < robot.lookahead) {
            if (distanceToFinal < 1.0) {
                println("Reached the end of the path ${path.last()} with distance $distanceToFinal (ending at ${robot.position})")
                return false
            }
            lastIntersection = Intersection(path.last(), pathSegments.last())
            canvas.setFill("#00FFFF")
                .fillCircle(lastIntersection.point.x, lastIntersection.point.y, 1.0)
            driveTo(path.last())
            drawRobot(canvas)
            ftcDashboard.sendTelemetryPacket(packet)
            return true
        }

        val remainingSegments = pathSegments.subList(lastSegment, pathSegments.size)
        val intersections = remainingSegments.flatMap { it.intersections(robot.lookaheadCircle) }
        // closest by angle from current heading
        val closestIntersection = intersections.minByOrNull { abs(getAngleDiff(it.point)) }
        for (intersection in intersections) {
            canvas.setFill(if (intersection == closestIntersection) "#00FF00" else "#FF0000")
                .fillCircle(intersection.point.x, intersection.point.y, 1.0)
        }
        val targetIntersection = closestIntersection ?: lastIntersection
        lastSegment = pathSegments.indexOf(targetIntersection.line)
        lastIntersection = targetIntersection
        driveTo(targetIntersection.point)

        drawRobot(canvas)

        ftcDashboard.sendTelemetryPacket(packet)
        return true
    }

    private fun drawRobot(canvas: ICanvas) {
        val lookaheadVector = Point(robot.lookahead, 0.0).rotate(robot.heading)
        val lookaheadPoint = robot.position + lookaheadVector

        canvas.setFill("#0000ff")
            .fillCircle(robot.position.x, robot.position.y, 1.0)
            .strokeCircle(robot.position.x, robot.position.y, robot.lookahead)
            .strokeLine(robot.position.x, robot.position.y, lookaheadPoint.x, lookaheadPoint.y)

        prevPosition.add(robot.position)
        canvas.setFill("#ffa500")
            .strokePolyline(
                prevPosition.map { it.x }.toDoubleArray(),
                prevPosition.map { it.y }.toDoubleArray()
            )
    }

    private fun driveTo(point: Point) {
        val angleDiff = getAngleDiff(point)
        val forwardPower = robot.position.distanceTo(point) / robot.lookahead
        drive(
            forwardPower,
            0.0,
            angleDiff
        ) // robot never strafes in pp since pp is a differential drive algorithm
    }

    private fun getAngleDiff(point: Point): Double {
        val angle = Utils.normalizeAngle(robot.position.angleTo(point))
        return Utils.normalizeAngle(angle - robot.heading)
    }
}