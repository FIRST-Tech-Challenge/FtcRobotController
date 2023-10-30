package computer.living.gamepadyn

class Configuration<T: Enum<T>>(vararg binds: ActionBind<T>) {
    internal var binds: ArrayList<ActionBind<T>>

    init {
        this.binds = arrayListOf(*binds)
    }
}