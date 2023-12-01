package computer.living.gamepadyn

/**
 * The state of any action is representable by an implementation of this interface.
 */
sealed class InputData {

    abstract val type: InputType

    final override fun equals(other: Any?): Boolean {
        if (other !is InputData) return false
        if (this::class == other::class) {
            if (this is InputDataDigital) {
                other as InputDataDigital

                return this.digitalData == other.digitalData
            } else if (this is InputDataAnalog) {
                other as InputDataAnalog

                if (this.axes != other.axes) return false
                return this.analogData.contentEquals(other.analogData)
            }
        }
        return false
    }
}

/**
 * Represents the value of a digital action.
 * This is effectively a Boolean that implements ActionData.
 */
class InputDataDigital(@JvmField var digitalData: Boolean = false): InputData() { override val type = InputType.DIGITAL  }

/**
 * Represents the value of an analog action.
 * @property analogData The action data. The size of the array is equal to the amount of axes the action has.
 */
class InputDataAnalog(dataFirst: Float?, vararg dataMore: Float?) : InputData() {
    override val type = InputType.ANALOG
    @JvmField val analogData: Array<Float?>

    val axes: Int
        get() { return analogData.size; }

    init {
        this.analogData = arrayOf(dataFirst, *dataMore)
    }
}