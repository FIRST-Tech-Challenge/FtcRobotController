@file:Suppress("HasPlatformType")

package ftc.rogue.blacksmith

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import kotlin.reflect.KClass
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
    /**
     * A Telemetry object which logs to both the driver station and the dashboard.
     */
    @JvmField
    protected val mTelemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

    init {
        _mTelemetry = mTelemetry
    }

    /**
     * A shorthand for hardwareMap, because I'm lazy and also just looks a bit cleaner for
     * longer declarations.
     */
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
        Scheduler.emit(STARTING_MSG)
        go()
    }

    companion object {
        private lateinit var _mTelemetry: MultipleTelemetry

        /**
         * Allows for a global telemetry object, akin to a global logger.
         */
        fun mTelemetry() = _mTelemetry

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
