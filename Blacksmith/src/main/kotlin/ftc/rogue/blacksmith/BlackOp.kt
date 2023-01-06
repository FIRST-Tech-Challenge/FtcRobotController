@file:Suppress("HasPlatformType")

package ftc.rogue.blacksmith

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import kotlin.reflect.KProperty

abstract class BlackOp : LinearOpMode() {
    @JvmField
    protected val mTelemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

    @JvmField
    protected var hwMap = hardwareMap

    abstract fun go()

    final override fun runOpMode() {
        Scheduler.reset()
        hwMap = hardwareMap
        Scheduler.emit(STARTING_MSG)
        go()
    }

    companion object {
        const val STARTING_MSG = 3248023743480398723L
    }

    // -- KOTLIN ONLY BELOW --

    protected inline fun <reified T> createOnGo(vararg args: () -> Any) =
        CreateOnStartR(T::class.java, *args)

    @JvmSynthetic
    protected fun <T> createOnGo(constructor: () -> T) =
        CreateOnStartL(constructor)

    // -- INTERNAL --

    private object Uninitialized

    protected inner class CreateOnStartR<T> @PublishedApi internal constructor(
        clazz: Class<T>,
        vararg args: () -> Any,
    ) {
        private var value: Any? = Uninitialized

        init {
            Scheduler.on(STARTING_MSG) {
                val invokedArgs = args.map { it() }.toTypedArray()
                val argTypes = invokedArgs.map { it::class.java }.toTypedArray()

                value = clazz.constructors.find { constructor ->
                    constructor.parameterTypes contentEquals argTypes
                }?.newInstance(*invokedArgs)
            }
        }

        operator fun getValue(thisRef: Any, property: KProperty<*>): T {
            if (value is Uninitialized) {
                throw IllegalStateException("Value is uninitialized (Not started yet)")
            }
            return value as T
        }
    }

    protected inner class CreateOnStartL<T> internal constructor(
        constructor: () -> T,
    ) {
        private var value: Any? = Uninitialized

        init {
            Scheduler.on(STARTING_MSG) {
                value = constructor()
            }
        }

        operator fun getValue(thisRef: Any, property: KProperty<*>): T {
            if (value is Uninitialized) {
                throw IllegalStateException("Value is uninitialized (Not started yet)")
            }
            return value as T
        }
    }
}
