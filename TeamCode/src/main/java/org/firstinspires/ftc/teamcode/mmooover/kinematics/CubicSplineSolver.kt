@file:SuppressLint("DefaultLocale")

package org.firstinspires.ftc.teamcode.mmooover.kinematics

import android.annotation.SuppressLint
import kotlin.math.abs
import kotlin.math.ceil
import kotlin.math.floor
import kotlin.math.max
import kotlin.math.min
import kotlin.math.pow
import kotlin.math.sqrt

const val PRECISION = 8

private inline fun DoubleArray.mapInPlace(transform: (Double) -> Double) {
    for (i in this.indices) {
        this[i] = transform(this[i])
    }
}

data class CubicSpline(
    val a: Double,
    val b: Double,
    val c: Double,
    val d: Double,
    val lowX: Double,
    val highX: Double,
) {
    companion object {
        @JvmStatic
        fun fromArrayAndBounds(arr: DoubleArray, lowX: Double, highX: Double): CubicSpline {
            assert(arr.size == 4) { "Wrong size to import cubic spline from array : ${arr.size}" }
            return CubicSpline(
                arr[0], arr[1], arr[2], arr[3], lowX, highX
            )
        }
    }

    fun at(x: Double): Double = a * x * x * x + b * x * x + c * x + d

    override fun toString() = "spline< ${toDesmos()} >"

    @JvmOverloads
    fun toDesmos(equation: Boolean = true, varSym: String = "x") = String.format(
        """%s%.${PRECISION}f$varSym^3 + %.${PRECISION}f$varSym^2 + %.${PRECISION}f$varSym + %.${PRECISION}f%s""",
        if (equation) {
            "y = "
        } else "", a, b, c, d,
        if (equation) {
            " \\left\\{%.${PRECISION}f \\leq $varSym \\leq %.${PRECISION}f\\right\\}".format(
                lowX, highX
            )
        } else ""
    )
}

data class CubicSplinePair(val x: CubicSpline, val y: CubicSpline) {
    data class PointData(val x: Double, val y: Double, val totalDistance: Double, val t: Double) {
        override fun toString(): String = "(%.4f, %.4f <t:%.4f | d:%.4f>)".format(x, y, t, totalDistance)
        fun toDesmos(): String = "(%.${PRECISION}f, %.${PRECISION}f)".format(x, y)
    }

    val tFrom = x.lowX
    val tTo = x.highX
    var pointCache: MutableList<PointData>? = null
    var totalDistance: Double = 0.0
    fun toDesmos() = "(${x.toDesmos(false, "t")}, ${y.toDesmos(false, "t")})"

    @JvmOverloads
    fun computeWaypoints(step: Double = 1.0, forceNoCache: Boolean = false, cacheResolution: Double = 0.01): List<PointData> {
        if (pointCache == null || forceNoCache) updateCache(cacheResolution)
        // save some not-null asserts later
        val pointCacheRef = pointCache ?: throw IllegalStateException("Point cache failed to compute for some reason and is still null")
        val fragments = ceil(totalDistance / step).toInt()

        var cursor = 0
        val waypoints: MutableList<PointData> = mutableListOf()
        for (fn in 0..fragments) {
            val position = fn.toDouble() / fragments // Percentage
            val positionUnits = position * totalDistance
            var currentAt = pointCacheRef[cursor++]
            while (currentAt.totalDistance < positionUnits && cursor < pointCacheRef.size) {
                currentAt = pointCacheRef[cursor++]
            }
            waypoints.add(currentAt)
        }
        val actualStep = totalDistance / fragments
        println("fragments: $fragments (%.4f per fragment, target %.4f, %.2f%% error)"
            .format(actualStep, step, (100 * (step - actualStep)) / step)
        )
        return waypoints
    }

    fun updateCache(resolution: Double = 0.01) {
        val newCache: MutableList<PointData> = mutableListOf()
        val steps = floor((tTo - tFrom) / resolution).toInt()
        var d = 0.0
        var lastX = 0.0
        var lastY = 0.0
        for (i in 0..steps) {
            val t = (tTo - tFrom) * (i.toDouble() / steps) + tFrom
            val xP = x.at(t)
            val yP = y.at(t)
            if (i > 0) {
                d += sqrt((xP - lastX).pow(2) + (yP - lastY).pow(2))
            }
            lastX = xP
            lastY = yP
            newCache.add(PointData(xP, yP, d, t))
        }
        pointCache = newCache
        totalDistance = d

        println(
            "total %.4f units | $steps steps".format(
                totalDistance
            )
        )
    }
}

object CubicSplineSolver {
    @JvmStatic
    fun solveMat(t1: Number, x1: Number, t2: Number, x2: Number, s1: Number, s2: Number) =
        solveMat(
            t1.toDouble(),
            x1.toDouble(),
            t2.toDouble(),
            x2.toDouble(),
            s1.toDouble(),
            s2.toDouble()
        )


    private fun helpPrintMatrix(arr: Array<DoubleArray>): String {
        return arr.map { it.toList().toString() }
            .joinToString("\n", prefix = "[\n", postfix = "\n]")
    }

    @JvmStatic
    fun solveMat(
        t1: Double,
        x1: Double,
        t2: Double,
        x2: Double,
        s1: Double,
        s2: Double,
    ): CubicSpline {
        var matrix: Array<DoubleArray> = arrayOf(
            doubleArrayOf(t1 * t1 * t1, t1 * t1, t1, 1.0, x1),
            doubleArrayOf(t2 * t2 * t2, t2 * t2, t2, 1.0, x2),
            doubleArrayOf(3 * t1 * t1, 2 * t1, 1.0, 0.0, s1),
            doubleArrayOf(3 * t2 * t2, 2 * t2, 1.0, 0.0, s2),
        )
        val nCols = matrix[0].size
        for ((rowN, row) in matrix.withIndex()) {
            val pivotCol = row.indexOfFirst { abs(it) > 1e-8 }
            if (pivotCol == -1 || pivotCol == row.size - 1) throw IllegalStateException("No suitable pivot column found")
            val coeff = row[pivotCol]
            row.mapInPlace { it -> it / coeff }
            for ((rowN2, row2) in matrix.withIndex()) {
                if (rowN2 == rowN) continue
                val multiplier = row2[pivotCol]
                for (colIdx in 0 until nCols) {
                    row2[colIdx] -= multiplier * row[colIdx]
                }
            }
        }

        val factors = DoubleArray(4)
        for (index in 0..3) {
            factors[index] = matrix.first {
                abs(it[index] - 1.0) < 0.001
            }.last()
        }
        val result = CubicSpline.fromArrayAndBounds(factors, min(t1, t2), max(t1, t2))
        println(result.toString())
        return result
    }

    private fun getSlope(x1: Double, x2: Double, y1: Double, y2: Double) = (y2 - y1) / (x2 - x1)

    @JvmStatic
    fun solveMultiSegment(xArr: DoubleArray, yArr: DoubleArray): List<CubicSpline> {
        assert(xArr.size == yArr.size) {
            "Mismatched coordinate sets: x has %d, y has %d".format(
                xArr.size,
                yArr.size
            )
        }
        assert(xArr.size >= 2) {
            "Not enough points (need at least 2)"
        }
        val m = DoubleArray(xArr.size)
        m[0] = getSlope(xArr[0], xArr[1], yArr[0], yArr[1])
        m[m.size - 1] = getSlope(
            xArr[xArr.size - 1],
            xArr[xArr.size - 2],
            yArr[yArr.size - 1],
            yArr[yArr.size - 2]
        )
        for (i in 1..m.size - 2) {
            m[i] = getSlope(
                xArr[i - 1],
                xArr[i + 1],
                yArr[i - 1],
                yArr[i + 1]
            )
        }
        val results: MutableList<CubicSpline> = mutableListOf()
        for (i in 0..<m.size - 1) results.add(
            solveMat(
                xArr[i], yArr[i], xArr[i + 1], yArr[i + 1],
                m[i], m[i + 1]
            )
        )
        return results
    }

    @JvmStatic
    fun solve2DMultiSegment(xArr: DoubleArray, yArr: DoubleArray): List<CubicSplinePair> {
        assert(xArr.size == yArr.size) {
            "Mismatched coordinate sets: x has %d, y has %d".format(
                xArr.size,
                yArr.size
            )
        }
        val pointCount = xArr.size
        assert(pointCount >= 2) {
            "Not enough points (need at least 2)"
        }
        val timescale = DoubleArray(pointCount)
        var lastT = 0.0
        for (i in 1..<pointCount) {
            lastT += sqrt((xArr[i] - xArr[i - 1]).pow(2) + (yArr[i] - yArr[i - 1]).pow(2))
            timescale[i] = lastT
        }
        val xSplines = solveMultiSegment(timescale, xArr)
        val ySplines = solveMultiSegment(timescale, yArr)
        val result: MutableList<CubicSplinePair> = mutableListOf()
        for (i in 0..<xSplines.size) {
            result.add(
                CubicSplinePair(
                    xSplines[i],
                    ySplines[i]
                )
            )
        }
        return result
    }
}