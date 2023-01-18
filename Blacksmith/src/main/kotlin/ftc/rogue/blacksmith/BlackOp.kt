@file:Suppress("HasPlatformType")

package ftc.rogue.blacksmith

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry
import kotlin.properties.Delegates
import kotlin.reflect.KProperty

/**
 * A nicer version of LinearOpMode, please read docs about this class if you wish to use it,
 * which I recommend, *especially* if you use Kotlin.
 *
 * Still nice for Java too.
 *
 * Provides some nice utilities for writing OpModes, such as:
 *  - mTelemetry which logs to both driver station and dashboard
 *  - hwMap, just short for hardwareMap
 *  - Emits a message (BlackOp.STARTING_MSG) which can be subscribed to do something with Scheduler.on
 *  - Allows use of `createOnGo` for Kotlin (read docs for this it is **very** nice)
 *  - Sounds kinda cool Idk
 *
 *  i like cars
 */
abstract class BlackOp : LinearOpMode() {
    @JvmField
    protected val mTelemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

    @JvmField
    protected var hwMap = hardwareMap

    init {
        Companion.mTelemetry = this.mTelemetry
    }

    /**
     * The method to override in place of runOpMode.
     *
     * goo goo ga ga.
     */
    abstract fun go()

    /**
     * Please override `go()` instead of this method.
     */
    final override fun runOpMode() {
        quickLog("Restarting Scheduler")
        Scheduler.reset()

        quickLog("Setting HwMap")
        hwMap = hardwareMap
        Companion.hwMap = hardwareMap

        quickLog("Emitting BS start msg")
        Scheduler.emit(STARTING_MSG)

        quickLog("Calling go")
        go()
    }

    protected fun quickLog(msg: String) {
        telemetry.addLine(msg)
        telemetry.update()
    }

    companion object {
        /**
         * A global telemetry holder, which logs to both the driver station and the dashboard.
         *
         * Is only assigned to a driver station telemetry when a BlackOp instance is initialized.
         *
         * **DO NOT CALL/USE DIRECTLY BEFORE A [BlackOp] INSTANCE IS INITIALIZED OR IT WON'T
         * BE THE RIGHT TELEMETRY**
         */
        @JvmStatic
        fun mTelemetry() = mTelemetry

        @get:JvmSynthetic
        var mTelemetry = telemetry
            private set

        /**
         * A global hardware map holder, and is also short for `hardwareMap`.
         *
         * Is only assigned to a hardware map when a BlackOp instance is initialized.
         *
         * **DO NOT CALL/USE DIRECTLY BEFORE A [BlackOp] INSTANCE IS INITIALIZED OR IT WON'T
         * BE THE RIGHT HARDWARE MAP**
         */
        @JvmStatic
        fun hwMap() = hwMap

        @get:JvmSynthetic
        var hwMap by Delegates.notNull<HardwareMap>()
            private set

        /**
         * The starting message emitted when runOpMode() is called on a BlackOp.
         *
         * ```java
         * Scheduler.on(BlackOp.STARTING_MSG, () -> {
         *    // do something when opmode starts
         * });
         */
        const val STARTING_MSG = 3248023743480398723L
    }

    // -- KOTLIN ONLY BELOW --

    /**
     * READ DOCS FOR THIS
     */
    @JvmSynthetic
    protected fun <T : Any> evalOnGo(constructor: () -> T) =
        CreateOnGo(constructor)

    /**
     * READ DOCS FOR THIS
     */
    protected inline fun <reified T : Any> createOnGo(noinline arg: () -> Any) =
        createOnGo<T>(*arrayOf(arg))

    /**
     * READ DOCS FOR THIS
     */
    protected inline fun <reified T : Any> createOnGo(vararg args: () -> Any) =
        CreateOnGo {
            val clazz = T::class.java

            val invokedArgs = args.map { it() }.toTypedArray()
            val argTypes = invokedArgs.map { it::class.java }.toTypedArray()

            val constructor = clazz.constructors.find { constructor ->
                constructor.parameterTypes contentEquals argTypes
            }

            constructor?.newInstance(*invokedArgs) as? T
                ?: throw FaultyCreationException("No constructor found for $clazz with args $argTypes")
        }

    // -- INTERNAL --

    protected inner class CreateOnGo<T : Any> @PublishedApi internal constructor(constructor: () -> T) {
        private lateinit var value: T

        init {
            Scheduler.on(STARTING_MSG) { value = constructor() }
        }

        operator fun getValue(thisRef: Any, property: KProperty<*>): T {
            if (!::value.isInitialized) {
                throw IllegalStateException("createOnGo value is uninitialized (OpMode not started yet)")
            }
            return value
        }
    }

    @PublishedApi
    internal class FaultyCreationException(message: String) : RuntimeException(message)
}
