package org.firstinspires.ftc.teamcode.mmooover.kinematics

class PathHelper internal constructor() {
    interface Keyframe
    interface XYKeyframe : Keyframe {
        val x: Double
        val y: Double
    }

    interface RotKeyframe : Keyframe {
        val r: Double
    }

    data class XYKeyframeImpl(override val x: Double, override val y: Double) : XYKeyframe
    data class RotKeyframeImpl(override val r: Double) : RotKeyframe
    data class ComboKeyframeImpl(
        override val x: Double,
        override val y: Double,
        override val r: Double
    ) : XYKeyframe, RotKeyframe

    val Number.deg get() = Math.toRadians(this.toDouble())

    val keyframes: MutableList<Keyframe> = mutableListOf()

    fun k(r: Number) = keyframes.add(RotKeyframeImpl(r.toDouble()))
    fun k(x: Number, y: Number) = keyframes.add(XYKeyframeImpl(x.toDouble(), y.toDouble()))
    fun k(x: Number, y: Number, r: Number) =
        keyframes.add(ComboKeyframeImpl(x.toDouble(), y.toDouble(), r.toDouble()))

    fun generate(): List<TimedWaypoint3> {
        assert(keyframes.size >= 2) { "Not enough keyframes (need at least 2, have ${keyframes.size})" }
        // Assert that the first and last keyframes are at least XY
        assert(keyframes[0] is XYKeyframe) { "First keyframe must have X/Y data" }
        assert(keyframes.last() is XYKeyframe) { "Last keyframe must have X/Y data" }

        val xyFrames: MutableList<XYKeyframe> = mutableListOf()
        val mapping: MutableList<Int> = mutableListOf()
        val isExact: MutableList<Boolean> = mutableListOf()
        val fractionals: MutableList<Double> = mutableListOf()

        var mapI = -1
        var splitCount = -1
        for (kf in keyframes) {
            if (kf is XYKeyframe) {
                mapping.add(++mapI)
                isExact.add(true)
                xyFrames.add(kf)
                fractionals.add(0.0)
                if (splitCount > 0) {
                    val onePart = 1.0 / (splitCount + 1)
                    for (i in 1 .. splitCount) {
                        fractionals[fractionals.size - i - 1] = 1 - (onePart * i)
                    }
                }
                splitCount = 0
            } else {
                mapping.add(mapI)
                isExact.add(false)
                fractionals.add(0.0)
                splitCount++
            }
        }

        // Generate the X/Y curves
        val xyCurves = CubicSplineSolver.solve2DMultiSegment(
            xyFrames.map { it.x }.toDoubleArray(),
            xyFrames.map { it.y }.toDoubleArray()
        )
        val timings = buildList {
            for (item in xyCurves) add(item.offset + item.tFrom)
            val last = xyCurves.last()
            add(last.offset + last.tTo)
        }
        val timing: MutableList<Double> = mutableListOf()
        val rotValues: MutableList<Double> = mutableListOf()

        for ((i, key) in keyframes.withIndex()) {
            if (key !is RotKeyframe) continue
            if (isExact[i]) {
                timing.add(timings[mapping[i]])
            } else {
                val span = timings[mapping[i]] - timings[mapping[i] - 1]
                val currentT = timings[mapping[i]-1] + span * fractionals[i]
                timing.add(currentT)
            }
            rotValues.add(key.r)
        }

        // Generate the rotation curves
        val rotCurves = CubicSplineSolver.solveMultiSegment(
            timing.toDoubleArray(),
            rotValues.toDoubleArray()
        )

        // Generate XY waypoints
        val waypointList: MutableList<CubicSplinePair.PointData> = mutableListOf()
        for (i in xyCurves) {
            if (!waypointList.isEmpty()) waypointList.removeAt(waypointList.lastIndex) // pop the duplicate waypoint
            waypointList.addAll(i.computeWaypoints())
        }

        // zip
        val zipped = CubicSplineSolver.thirdChannel(waypointList, rotCurves)
        return zipped
    }
}

fun path(configure: PathHelper.() -> Unit): PathHelper {
    val target = PathHelper()
    target.configure()
    return target
}