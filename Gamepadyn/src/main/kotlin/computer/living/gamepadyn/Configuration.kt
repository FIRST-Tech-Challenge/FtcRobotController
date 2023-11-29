package computer.living.gamepadyn

data class Configuration<T: Enum<T>>(var binds: ArrayList<ActionBind<T>>) {
    constructor(vararg binds: ActionBind<T>) : this(arrayListOf(*binds))
}