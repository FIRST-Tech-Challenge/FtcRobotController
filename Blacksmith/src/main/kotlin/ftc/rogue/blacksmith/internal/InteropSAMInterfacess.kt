package ftc.rogue.blacksmith.internal

import ftc.rogue.blacksmith.Anvil
import ftc.rogue.blacksmith.internal.anvil.AnvilRunConfig

fun interface AnvilRunConfigBuilder {
    fun AnvilRunConfig.build()
}

typealias AnvilConsumer = Consumer<Anvil>

typealias DoubleConsumer = Consumer<Double>

fun interface Consumer<T> {
    fun T.consume()
}

@JvmSynthetic
fun <T> Consumer<T>.consume(scope: T) {
    scope.consume()
}

fun interface AnvilCycle {
    fun Anvil.doCycle(iteration: Int)
}

@JvmSynthetic
fun AnvilCycle.consume(instance: Anvil, iteration: Int) {
    instance.doCycle(iteration)
}

