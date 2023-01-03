package ftc.rogue.blacksmith.units

import java.io.BufferedReader
import java.io.BufferedWriter
import java.io.File
import java.io.FileReader
import java.io.FileWriter
import java.util.*

object GlobalUnits {
    private const val SAVE_PATH = ".blacksmith/units.properties"

    lateinit var distance: DistanceUnit
        private set
    lateinit var angle: AngleUnit
        private set
    lateinit var time: TimeUnit
        private set

    init {
//        loadUnits()
        distance = DistanceUnit.CENTIMETERS
        angle = AngleUnit.DEGREES
        time = TimeUnit.MILLISECONDS
    }

    @JvmStatic
    fun distance() = distance

    @JvmStatic
    fun angle() = angle

    @JvmStatic
    fun time() = time

    @JvmStatic
    @JvmOverloads
    fun setUnits(
        distanceUnit: DistanceUnit = distance,
        angleUnit: AngleUnit = angle,
        timeUnit: TimeUnit = time,
    ): Unit = Properties().run {
        distance = distanceUnit
        angle = angleUnit
        time = timeUnit
    }

    private fun loadUnits() = Properties().run {
        File(SAVE_PATH).createNewFile()

        BufferedReader(FileReader(SAVE_PATH)).use(::load)

        val d = getProperty("distance") ?: "INCHES".also { setProperty("distance", it) }
        val a = getProperty("angle") ?: "radians".also { setProperty("angle", it) }
        val t = getProperty("time") ?: "seconds".also { setProperty("time", it) }

        distance = enumValueOf(d.uppercase())
           angle = enumValueOf(a.uppercase())
            time = enumValueOf(t.uppercase())

        BufferedWriter(FileWriter(SAVE_PATH)).use { store(it, null) }
    }
}
