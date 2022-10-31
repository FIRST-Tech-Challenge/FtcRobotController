package org.firstinspires.ftc.teamcode.koawalib

import com.asiankoala.koawalib.logger.Logger
import com.asiankoala.koawalib.math.Pose
import com.asiankoala.koawalib.math.Vector
import com.asiankoala.koawalib.math.angleWrap
import com.asiankoala.koawalib.math.epsilonEquals
import kotlin.math.PI

class DriveHack(
    private val ps: () -> Pose,
    private val R: Double,
    private val N: Double,
    private val hp: Double,
    private val obstacles: List<Vector> = OBSTACLES,
) {

    fun spaceglide(i: Vector): Vector {
        val p = ps.invoke().vec
        val o = obstacles.minByOrNull { (p - it).norm }!!
        Logger.addTelemetryData("closest obstacle", o)
        val r = p - o
        val v1 = i.unit
        val theta = PI / 2.0
        val d1 = r dot v1
        val rm = r.norm
        val a = N / rm
        val c = r cross v1
        val n2 = r.rotate(if (c >= 0) theta else -theta)
        val v2 = if (d1 >= 0.0 || r.norm > R) v1 else n2 + r * a
        val im = i.norm
        val u = v2.unit * im
        Logger.addTelemetryData("output", u)
        return u
    }

    fun aimbot(i: Vector): Double {
        val p = ps.invoke()
        val xn = (p.x.toInt() / 24)
        val yn = (p.y.toInt() / 24)
        val c = Vector(xn * 24.0 + 12.0, yn * 24.0 + 12.0)
        val os = listOf(
            Vector(c.x + 12.0, c.y + 12.0),
            Vector(c.x + 12.0, c.y - 12.0),
            Vector(c.x - 12.0, c.y + 12.0),
            Vector(c.x - 12.0, c.y - 12.0)
        )
        val o = os.maxByOrNull { (it - p.vec) dot i }!!
        val a = (o - p.vec).angle
        val dh = (p.heading - a).angleWrap
        return dh / hp
    }

    companion object {
        private val OBSTACLES = listOf(
            Vector(-48.0, -24.0),
            Vector(-24.0, -48.0),
            Vector(-24.0, -24.0)
//            Vector(-48.0, -48.0),
//            Vector(-48.0, -24.0),
//            Vector(-48.0, 0.0),
//            Vector(-48.0, 24.0),
//            Vector(-48.0, 48.0),
//            Vector(-24.0, -48.0),
//            Vector(-24.0, -24.0),
//            Vector(-24.0, 0.0),
//            Vector(-24.0, 24.0),
//            Vector(-24.0, 48.0),
//            Vector(0.0, -48.0),
//            Vector(0.0, -24.0),
//            Vector(0.0, 0.0),
//            Vector(0.0, 24.0),
//            Vector(0.0, 48.0),
//            Vector(24.0, -48.0),
//            Vector(24.0, -24.0),
//            Vector(24.0, 0.0),
//            Vector(24.0, 24.0),
//            Vector(24.0, 48.0),
//            Vector(48.0, -48.0),
//            Vector(48.0, -24.0),
//            Vector(48.0, 0.0),
//            Vector(48.0, 24.0),
//            Vector(48.0, 48.0),
//            Vector(-12.0, -70.0),
//            Vector(-12.0, 70.0),
//            Vector(12.0, -70.0),
//            Vector(12.0, 70.0),
//            Vector(-36.0, -36.0),
//            Vector(-36.0, 36.0),
//            Vector(36.0, -36.0),
//            Vector(36.0, 36.0),
        )
    }
}