package computer.living.gamepadyn

import computer.living.gamepadyn.ActionType.ANALOG
import computer.living.gamepadyn.ActionType.DIGITAL
import javax.validation.constraints.PositiveOrZero

@Target(AnnotationTarget.VALUE_PARAMETER)
annotation class MustBeAnalog
@Target(AnnotationTarget.VALUE_PARAMETER)
annotation class MustBeDigital

sealed interface ActionBind<T: Enum<T>> { val type: ActionType; val bindMask: Int; }
sealed class ActionBindAnalog<T: Enum<T>> : ActionBind<T> { override val type: ActionType = ANALOG; }
sealed class ActionBindDigital<T: Enum<T>> : ActionBind<T> { override val type: ActionType = DIGITAL; }
enum class ActionBindMask(val mask: Int) {
    DIGITAL_PRESS(1),
    DIGITAL_RELEASE(2),
    FULL(Int.MAX_VALUE)
}

class ActionBindDigitalToDigital<T: Enum<T>>(
    /**
     * A digital input (axes must be 1 or 0)
     */
    @MustBeDigital internal var sourceInput: RawInput,
    /**
     * The digital action that this binding refers to
     */
    @MustBeDigital internal var targetAction: T,
) : ActionBindDigital<T>() {
    override val bindMask: Int = (ActionBindMask.DIGITAL_PRESS.mask or ActionBindMask.DIGITAL_RELEASE.mask)

    init {
        assert(sourceInput.axes == 0 || sourceInput.axes == 1)
    }
}
class ActionBindAnalogToAnalog<T: Enum<T>>(
    /**
     * An analog input
     */
    @MustBeAnalog internal var sourceInput: RawInput,
    /**
     * The analog action that this binding refers to
     */
    @MustBeAnalog internal var targetAction: T
) : ActionBindAnalog<T>() {
    override val bindMask: Int = ActionBindMask.FULL.mask

    init {
        assert(sourceInput.axes > 0)
        // would like to assert information about the action but it would be difficult
    }
}
class ActionBindDigitalHoldToAnalogRate<T: Enum<T>>(
    /**
     * A digital input (axes must be 1 or 0)
     */
    @MustBeDigital internal var sourceInput: RawInput,
    /**
     * The analog action that this binding refers to
     */
    @MustBeAnalog internal var targetAction: T,
    /**
     * The axis of the analog action (must be >= 0)
     */
    @PositiveOrZero internal var targetAxis: Int,
    /**
     * Update rate, measured in units per second
     */
    internal var rate: Float
) : ActionBindAnalog<T>() {
    override val bindMask: Int = ActionBindMask.DIGITAL_PRESS.mask

    init {
        assert(sourceInput.axes == 0 || sourceInput.axes == 1)
        assert(targetAxis >= 0)
    }
}
