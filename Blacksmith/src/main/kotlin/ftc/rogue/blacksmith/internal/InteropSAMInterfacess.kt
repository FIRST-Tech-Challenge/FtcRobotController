package ftc.rogue.blacksmith.internal

import ftc.rogue.blacksmith.Anvil
import ftc.rogue.blacksmith.internal.anvil.AnvilRunConfig

fun interface AnvilRunConfigBuilder {
    fun AnvilRunConfig.build()
}

typealias AnvilConsumer = Consumer<Anvil>

fun interface Consumer<T> {
    fun T.consume()
}

fun interface AnvilCycle {
    fun Anvil.doCycle(iteration: Int)
}

typealias BuilderAction = () -> Unit
