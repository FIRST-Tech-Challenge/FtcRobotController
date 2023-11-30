package computer.living.gamepadyn

@Suppress("unused", "unused", "unused", "unused")
open class ActionBind<T: Enum<T>>(val input: RawInput, internal val targetAction: T) {

    /**
     * Performs a transformation on the input data. The return value of this function
     */
    open fun transform(data: InputData, targetDescriptor: ActionDescriptor): InputData = data

}