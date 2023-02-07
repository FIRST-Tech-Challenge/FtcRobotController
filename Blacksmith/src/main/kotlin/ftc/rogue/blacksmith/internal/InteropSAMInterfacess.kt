package ftc.rogue.blacksmith.internal

import ftc.rogue.blacksmith.Anvil

fun interface AnvilConfigBuilder {
    fun AnvilRunConfig.build()
}

typealias AnvilConsumer = Consumer<Anvil>

fun interface Consumer<T> {
    fun T.consume()
}

fun interface AnvilCycle {
    fun Anvil.doCycle(iteration: Int)
}

fun interface BuilderAction {
    operator fun invoke()
}
