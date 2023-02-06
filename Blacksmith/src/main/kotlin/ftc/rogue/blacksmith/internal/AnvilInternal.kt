package ftc.rogue.blacksmith.internal

import ftc.rogue.blacksmith.Anvil
import ftc.rogue.blacksmith.Scheduler

class AnvilRunConfig @PublishedApi internal constructor() {
    @PublishedApi @JvmSynthetic internal var buildsSynchronously = false
        private set

    @PublishedApi @JvmSynthetic internal var runsSynchronously = false
        private set

    @PublishedApi @JvmSynthetic internal var predicate = { true }
        private set

    fun buildSynchronously(): AnvilRunConfig {
        buildsSynchronously = true
        return this
    }

    fun runSynchronously(): AnvilRunConfig {
        runsSynchronously = true
        return this
    }

    fun onCondition(condition: () -> Boolean): AnvilRunConfig {
        predicate = condition
        return this
    }
}

class AnvilLaunchConfig1 internal constructor() {
    private var async = true

    fun onSchedulerLaunch(): AnvilLaunchConfig2 {
        Scheduler.on(Scheduler.STARTING_MSG) { Anvil.start(async) }
        return AnvilLaunchConfig2()
    }

    inner class AnvilLaunchConfig2 internal constructor() {
        fun synchronously() { async = false }
    }
}
