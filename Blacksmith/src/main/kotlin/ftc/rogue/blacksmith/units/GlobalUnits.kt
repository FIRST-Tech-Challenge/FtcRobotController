package ftc.rogue.blacksmith.units

import android.content.Context
import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import java.io.BufferedReader
import java.io.File
import java.io.FileReader
import java.util.*
import kotlin.reflect.KProperty

object GlobalUnits {
    var distance by SetUnitFirst<DistanceUnit>()
        private set
    var angle by SetUnitFirst<AngleUnit>()
        private set
    var time by SetUnitFirst<TimeUnit>()
        private set

    init {
        distance = DistanceUnit.CENTIMETERS
        angle = AngleUnit.DEGREES
        time = TimeUnit.SECONDS
        loadSavedUnits()
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

    private const val RES_SAVE_PATH = "blacksmith/units.properties"
    private const val FULL_SAVE_PATH = "./TeamCode/src/main/res/raw/blacksmith/units.properties"

    private fun loadSavedUnits() {
        if (File(FULL_SAVE_PATH).exists()) {
            loadUnitsFromFullPath()
            return
        }

        try {
            val ctx = AppUtil.getDefContext().applicationContext

            AppUtil.getInstance().isDriverStation

            val id = ctx.resources.getIdentifier(RES_SAVE_PATH, "raw", ctx.packageName)

            if (id != 0) {
                loadUnitsFromRes(ctx, id)
            }
        } catch (_: ExceptionInInitializerError) {
        }
    }

    private fun loadUnitsFromRes(ctx: Context, id: Int) = Properties().run {
        val stream = ctx.resources.openRawResource(id)

        stream.bufferedReader().use(::load)

        setUnitsFromProperty(this)
    }

    private fun loadUnitsFromFullPath() = Properties().run {
        val reader = FileReader(FULL_SAVE_PATH)

        BufferedReader(reader).use(::load)

        setUnitsFromProperty(this)
    }

    private fun setUnitsFromProperty(properties: Properties) = properties.run {
        val d = getProperty("distance") ?: "inches".also { setProperty("distance", it) }
        val a = getProperty("angle") ?: "radians".also { setProperty("angle", it) }
        val t = getProperty("time") ?: "seconds".also { setProperty("time", it) }

        distance = enumValueOf(d.uppercase())
           angle = enumValueOf(a.uppercase())
            time = enumValueOf(t.uppercase())
    }

    private class SetUnitFirst<T : Any> {
        lateinit var value: T

        operator fun getValue(thisRef: Any, property: KProperty<*>): T {
            if (!::value.isInitialized) {
                throw IllegalStateException("Set GlobalUnits first (pls check blacksmith docs for more info)")
            }
            return value
        }

        operator fun setValue(thisRef: Any, property: KProperty<*>, _value: T) {
            value = _value
        }
    }
}
