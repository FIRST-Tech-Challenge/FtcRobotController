@file:Suppress("HasPlatformType")

package ftc.rogue.blacksmith

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import ftc.rogue.blacksmith.annotations.CreateOnStart
import ftc.rogue.blacksmith.util.getFieldsAnnotatedWith
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
abstract class BlackOp() : LinearOpMode() {
    @JvmField
    protected val mTelemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

    @JvmField
    protected var hwMap = hardwareMap

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
        Scheduler.reset()

        hwMap = hardwareMap

        Companion.hwMap = hardwareMap
        Companion.mTelemetry = mTelemetry

        tast()

        Scheduler.emit(STARTING_MSG)

        this::class.java
            .getFieldsAnnotatedWith(CreateOnStart::class.java)
            .forEach {
                it.set(this, it.type.getConstructor().newInstance())
            }

        go()
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
        var mTelemetry by Delegates.notNull<MultipleTelemetry>()
            private set

        /**
         * A global hardware map holder, and is also short for `hardwareMap`.
         *
         * Is only assigned to a hardware map when a BlackOp instance is initialized.
         *
         * **DO NOT CALL/USE DIRECTLY BEFORE A [BlackOp] INSTANCE IS INITIALIZED OR IT WON'T
         * BE THE RIGHT HARDWARE MAP**
         */
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
        createOnGo<T>(*arrayOf(arg)) // Need to do spread on array or type checker errors

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
                ?: throw CreationException("No constructor found for $clazz with args $argTypes")
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
    internal class CreationException(message: String) : RuntimeException(message)
}
