package computer.living.gamepadyn

class Configuration<T: Enum<T>> {
    internal var binds = ArrayList<ActionBind<T>>()
//    fun addBind(): Configuration<T>? {
//
//        return this
//    }
}