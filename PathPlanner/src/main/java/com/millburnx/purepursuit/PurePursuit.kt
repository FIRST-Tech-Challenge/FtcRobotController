package com.millburnx.purepursuit

import com.millburnx.dashboard.ICanvas
import com.millburnx.dashboard.ITelemetryPacket
import com.millburnx.utils.Bezier
import com.millburnx.utils.Circle
import com.millburnx.utils.Intersection
import com.millburnx.utils.Utils
import com.millburnx.utils.Vec2d
import java.awt.Color
import kotlin.math.abs

class PurePursuit(
    path: List<Vec2d>,
    val lookahead: Double,
    val threshold: Double = 1.0,
) {
    private val bezier: List<Bezier> = Utils.pathToBeziers(path)
    private val lastPoint: Vec2d = path.last()
    private var lastSegment: Int = 0
    private var lastIntersection: Intersection<Bezier> = Intersection(path[0], bezier[0])
    var isFinished = false

    fun getIntersections(lookahead: Circle, segments: List<Bezier>): List<Intersection<Bezier>> {
        return segments.flatMap { it.intersections(lookahead) }
    }

    fun getTarget(
        intersections: List<Intersection<Bezier>>,
        pos: Vec2d,
        heading: Double
    ): Intersection<Bezier> {
        val closest = intersections.minByOrNull {
            abs(getAngleDiff((pos to heading), it.point))
        }
        return closest ?: lastIntersection
    }

    fun updateLastIntersection(lastIntersection: Intersection<Bezier>) {
        this.lastIntersection = lastIntersection
        this.lastSegment = bezier.indexOf(lastIntersection.line)
    }

    fun calc(pos: Vec2d, heading: Double): PurePursuitData {
        val distanceToLast = pos.distanceTo(lastPoint)
        if (distanceToLast < threshold) {
            isFinished = true

            @Suppress("KotlinConstantConditions")
            return PurePursuitData(
                lastIntersection.point,
                isFinished,
                bezier,
                listOf(),
                listOf(lastIntersection)
            )
        }
        if (distanceToLast <= lookahead) {
            return PurePursuitData(
                lastPoint,
                isFinished,
                bezier,
                listOf(Bezier.fromLine(pos, lastPoint)),
                listOf(lastIntersection)
            )
        }
        val lookaheadCircle = Circle(pos, lookahead)
        val remainingPath = bezier.subList(lastSegment, bezier.size)
        val intersections = getIntersections(lookaheadCircle, remainingPath)
        val targetIntersection = getTarget(intersections, pos, heading)
        updateLastIntersection(targetIntersection)

        return PurePursuitData(
            targetIntersection.point,
            isFinished,
            bezier,
            remainingPath,
            intersections
        )
    }

    companion object {
        fun getAngleDiff(a: Pair<Vec2d, Double>, b: Vec2d): Double {
            val (aPoint, aAngle) = a
            val diff = Utils.normalizeAngle(aPoint.angleTo(b))
            return Utils.normalizeAngle(diff - aAngle)
        }

        fun render(data: PurePursuitData, packet: ITelemetryPacket, addTelemetry: Boolean = true) {
            val (target, isDone, path, remainingPath, intersections) = data
            if (addTelemetry) {
                packet.put("pure_pursuit/target", target)
                packet.put("pure_pursuit/is_done", isDone)
                packet.put("pure_pursuit/path", path)
                packet.put("pure_pursuit/remaining_path", remainingPath)
                packet.put("pure_pursuit/intersections", intersections)
            }
            val canvas = packet.fieldOverlay()
            val colors = listOf(Utils.Colors.red, Utils.Colors.blue, Utils.Colors.green, Utils.Colors.yellow)
            renderPath(canvas, path, colors)
            renderIntersections(canvas, intersections, target, colors)
        }

        fun renderPath(canvas: ICanvas, path: List<Bezier>, colors: List<Color>) {
            var color = 0
            for (bezier in path) {
                val currentColor = colors[color % colors.size]
                val colorString = String.format("#%06x", currentColor.rgb)
                canvas.setStroke(colorString)
                bezier.draw(canvas)
                color++
            }
        }

        fun renderIntersections(
            canvas: ICanvas,
            intersections: List<Intersection<Bezier>>,
            target: Vec2d,
            colors: List<Color>
        ) {
            var color = 0
            for (intersection in intersections) {
                val currentColor = if (intersection.point == target) {
                    colors[color % colors.size]
                } else {
                    Color.WHITE
                }
                val colorString = String.format("#%06x", currentColor.rgb)
                canvas.setFill(colorString)
                    .fillCircle(intersection.point.x, intersection.point.y, 1.0)
                color++
            }
        }
    }
}

data class PurePursuitData(
    val target: Vec2d,
    val isFinished: Boolean,
    val path: List<Bezier>,
    val remainingPath: List<Bezier>,
    val intersections: List<Intersection<Bezier>>,
)