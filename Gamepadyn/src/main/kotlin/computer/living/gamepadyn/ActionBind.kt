package computer.living.gamepadyn

open class ActionBind<T: Enum<T>>(val input: RawInput, internal val targetAction: T) {

    /**
     * Performs a transformation on the input data.
     *
     * NOTE: the InputData parameter should match the InputDescriptor of the `input` field, but under some circumstances, it may not. You should still assume they will match.
     * @param targetDescriptor The return value of this function must conform with this parameter's descriptor.
     * @return the result of the transformation.
     */
    open fun transform(data: InputData, targetDescriptor: InputDescriptor): InputData = data

}