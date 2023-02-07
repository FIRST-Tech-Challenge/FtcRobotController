package ftc.rogue.blacksmith.units

import android.content.Context
import ftc.rogue.blacksmith.util.toIn
import ftc.rogue.blacksmith.util.toRad
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import org.firstinspires.ftc.ftccommon.external.OnCreate
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

    @JvmStatic
    @JvmOverloads
    fun pos(x: Number = 0, y: Number = 0, heading: Number = 0): Pose2d {
        return Pose2d(x.toIn(), y.toIn(), heading.toRad())
    }

    @JvmStatic
    @JvmOverloads
    fun vec(x: Number = 0, y: Number = 0): Vector2d {
        return Vector2d(x.toIn(), y.toIn())
    }

    @JvmStatic
    fun pos(pose2d: Pose2d): Pose2d {
        return Pose2d(pose2d.x.toIn(), pose2d.y.toIn(), pose2d.heading.toRad())
    }

    @JvmStatic
    fun vec(vector2d: Vector2d): Vector2d {
        return Vector2d(vector2d.x.toIn(), vector2d.y.toIn())
    }

    private const val UNITS_FILE_NAME = "bsm_units"
    private const val FULL_SAVE_PATH = "./TeamCode/src/main/res/raw/$UNITS_FILE_NAME.properties"

    init {
        if (File(FULL_SAVE_PATH).exists()) {
            loadUnitsFromFullPath()
        } else try {
            val ctx = AppUtil.getDefContext().baseContext

            val id = ctx.resources.getIdentifier(UNITS_FILE_NAME, "raw", ctx.packageName)

            if (id != 0) {
                loadUnitsFromRes(ctx, id)
            }
        } catch (_: ExceptionInInitializerError) {}
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
