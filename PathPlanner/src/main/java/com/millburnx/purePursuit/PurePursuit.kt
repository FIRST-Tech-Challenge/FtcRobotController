package com.millburnx.purePursuit

import com.millburnx.purePursuit.Utils.*
import com.millburnx.purePursuit.ftcDashboard.ICanvas
import com.millburnx.purePursuit.ftcDashboard.TelemetryPacket
import java.awt.FileDialog
import java.io.File
import kotlin.math.abs

class PurePursuit(ppi: Double, updateHertz: Double = -1.0) : OpMode(ppi, updateHertz) {
    private var path: List<Vec2d> = listOf(
        Vec2d(0.0, 0.0),
        Vec2d(48.0, 0.0),
        Vec2d(48.0, -48.0),
        Vec2d(0.0, -48.0),
    )

    private var beziers: List<Bezier> = Utils.pathToBeziers(path)

    private var lastIntersection: Intersection<Bezier> = Intersection(path[0], beziers[0]) // start of the path
    private var lastSegment: Int = 0 // prevent backtracking

    private val prevPositions: MutableList<Vec2d> = mutableListOf()

    override fun init() {
        println("Initializing Pure Pursuit")
        ftcDashboard.reset = {
            stop()
            robot.position = Vec2d(0.0, 0.0)
            robot.heading = 0.0
            prevPositions.clear()
            lastSegment = 0
            lastIntersection = Intersection(path[0], beziers[0])
            lastFrame = 0L
            val packet = TelemetryPacket()
            val canvas = packet.fieldOverlay()
            renderPath(canvas)
            drawRobot(canvas)
            ftcDashboard.sendTelemetryPacket(packet)
        }
        ftcDashboard.load = {
            stop()
            loadPath()
        }
    }

    fun updatePath() {
        beziers = Utils.pathToBeziers(path)
        lastSegment = 0
        lastIntersection = Intersection(path[0], beziers[0])
        val packet = TelemetryPacket()
        val canvas = packet.fieldOverlay()
        renderPath(canvas)
        drawRobot(canvas)
        ftcDashboard.sendTelemetryPacket(packet)
    }

    fun loadPath() {
        val fileDialog = FileDialog(null as java.awt.Frame?, "Select a file", FileDialog.LOAD)
        fileDialog.directory = File("paths").absolutePath
        fileDialog.file = "*.tsv"
        fileDialog.isVisible = true
        val file = fileDialog.file
        if (file == null) {
            println("No file selected")
            return
        }
        val pathFile = File(fileDialog.directory, file)
        val pathList = Vec2d.loadList(pathFile)
        path = pathList
        updatePath()
    }

    fun renderPath(canvas: ICanvas) {
        canvas.drawImage("PathPlanner/src/main/resources/bg.png", 0.0, 0.0, 144.0, 144.0)
//        canvas
            .setFill("#808080")
            .drawGrid(0.0, 0.0, 144.0, 144.0, 7, 7)
            .setFill("#D3D3D3")
            .setStrokeWidth(2)
        for (bezier in beziers) {
            bezier.draw(canvas)
        }
//        for (point in path) {
//            canvas.fillCircle(point.x, point.y, 1.0)
//        }
        var color = 0
        val anchorColor = "#ffffff"
        val prevHandleColor = "#ff00ff"
        val nextHandleColor = "#00ff00"
        val colors = listOf(anchorColor, nextHandleColor, prevHandleColor)
        for ((index, point) in path.withIndex()) {
            var colorString = colors[color]
            color = (color + 1) % colors.size
            canvas.setFill(colorString)
                .fillCircle(point.x, point.y, 1.0)
//                .fillText(index.toString(), point.x, point.y, "4px FreeSans", 0.0, false)
        }
    }

    override fun loop(): Boolean {
        val packet = TelemetryPacket()
        val canvas = packet.fieldOverlay()

        renderPath(canvas)

        val distanceToFinal = robot.position.distanceTo(path.last())
//        println(distanceToFinal)
        if (distanceToFinal < robot.lookahead) {
            if (distanceToFinal < 1.0) {
                println("Reached the end of the path ${path.last()} with distance $distanceToFinal (ending at ${robot.position})")
                return false
            }
            lastIntersection = Intersection(path.last(), beziers.last())
            canvas.setFill("#00FFFF")
                .fillCircle(lastIntersection.point.x, lastIntersection.point.y, 1.0)
            driveTo(path.last())
            drawRobot(canvas)
            ftcDashboard.sendTelemetryPacket(packet)
            return true
        }

        val remainingSegments = beziers.subList(lastSegment, beziers.size)
        val intersections = remainingSegments.flatMap { it.intersections(robot.lookaheadCircle) }
        // closest by angle from current heading
        val closestIntersection = intersections.minByOrNull { abs(getAngleDiff(it.point)) }
        for (intersection in intersections) {
            canvas.setFill(if (intersection == closestIntersection) "#00FF00" else "#FF0000")
                .fillCircle(intersection.point.x, intersection.point.y, 1.0)
        }
        val targetIntersection = closestIntersection ?: lastIntersection
        lastSegment = beziers.indexOf(targetIntersection.line)
        lastIntersection = targetIntersection
        driveTo(targetIntersection.point)

        drawRobot(canvas)

        ftcDashboard.sendTelemetryPacket(packet)
        return true
    }

    private fun drawRobot(canvas: ICanvas) {
        val lookaheadVector = Vec2d(robot.lookahead, 0.0).rotate(robot.heading)
        val lookaheadPoint = robot.position + lookaheadVector

        canvas.setFill("#0000ff")
            .fillCircle(robot.position.x, robot.position.y, 1.0)
            .strokeCircle(robot.position.x, robot.position.y, robot.lookahead)
            .strokeLine(robot.position.x, robot.position.y, lookaheadPoint.x, lookaheadPoint.y)

        prevPositions.add(robot.position)
        val copyPositions = prevPositions.toList()
        canvas.setFill("#ffa500")
            .strokePolyline(
                copyPositions.map { it.x }.toDoubleArray(),
                copyPositions.map { it.y }.toDoubleArray()
            )
    }

    private fun driveTo(point: Vec2d) {
        val angleDiff = getAngleDiff(point)
        val forwardPower = robot.position.distanceTo(point) / robot.lookahead
        drive(
            forwardPower,
            0.0,
            angleDiff
        ) // robot never strafes in pp since pp is a differential drive algorithm
    }

    private fun getAngleDiff(point: Vec2d): Double {
        val angle = Utils.normalizeAngle(robot.position.angleTo(point))
        return Utils.normalizeAngle(angle - robot.heading)
    }
}