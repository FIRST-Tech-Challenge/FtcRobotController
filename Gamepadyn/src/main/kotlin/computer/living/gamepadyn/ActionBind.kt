package computer.living.gamepadyn

open class ActionBind<T: Enum<T>>(val input: RawInput, internal val targetAction: T) {

    /**
     * Performs a transformation on the input data.
     */
    open fun transform(data: InputData, targetDescriptor: ActionDescriptor): InputData {
        return data
    }

//    /**
//     * Specialization functions.
//     */
//    open fun transformAnalog(input: RawInput, data: InputDataAnalog): InputData = transform(input, data)
//    open fun transformDigital(input: RawInput, data: InputDataDigital): InputData = transform(input, data)
}