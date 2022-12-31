//package org.firstinspires.ftc.teamcode.koawalib
//
//import com.asiankoala.koawalib.math.*
//import kotlin.math.PI
//
//class DriveHack(
//    private val ps: () -> Pose,
//    private val R: Double,
//    private val N: Double,
//    private val H: Double,
//    private val obstacles: List<Vector> = OBSTACLES,
//) {
//    // https://www.desmos.com/calculator/vq9ykp5m3r
//    fun spaceglide(i: Vector): NVector {
//        val p = ps.invoke().vec
//        val o = obstacles.minByOrNull { (p - it).norm }!!
//        val r = p - o
//        val v1 = i.unit
//        val theta = PI / 2.0
//        val d1 = r dot v1
//        val rm = r.norm
//        val a = N / rm
//        val c = r cross v1
//        val n2 = r.rotate(if (c >= 0) theta else -theta)
//        val v2 = if (d1 >= 0.0 || r.norm > R) v1 else n2 + r * a
//        val im = i.norm
//        val u = v2.unit.asN
//        return u * im
//    }
//
//    fun aimbot(i: Vector): Double {
//        val p = ps.invoke()
//        val xn = (p.x.toInt() / 24)
//        val yn = (p.y.toInt() / 24)
//        val c = Vector(xn * 24.0 + 12.0, yn * 24.0 + 12.0)
//        val os = listOf(
//            Vector(c.x + 12.0, c.y + 12.0),
//            Vector(c.x + 12.0, c.y - 12.0),
//            Vector(c.x - 12.0, c.y + 12.0),
//            Vector(c.x - 12.0, c.y - 12.0)
//        )
//        val o = os.maxByOrNull { (it - p.vec) dot i }!!
//        val a = (o - p.vec).angle
//        val dh = (p.heading - a).angleWrap
//        return dh / H
//    }
//
//    companion object {
//        private val OBSTACLES = listOf(
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
//        )
//    }
//}