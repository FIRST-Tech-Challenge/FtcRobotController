package ftc.rogue.blacksmith.internal.anvil

import ftc.rogue.blacksmith.Anvil
import ftc.rogue.blacksmith.Scheduler

class AnvilRunner internal constructor() {
    private lateinit var initialTrajectory: Any
    private lateinit var initialInstance: Anvil

    private var async = true

    @JvmOverloads
    fun start(async: Boolean = true) {
        if (!::initialTrajectory.isInitialized) {
            throw IllegalStateException("Anvil.startAutoWith() should be called before Anvil.start()")
        }
        initialInstance.internal.run(initialTrajectory, async)
    }

    fun onSchedulerLaunch() = this.also {
        Scheduler.on(Scheduler.STARTING_MSG) { start(async) }
    }

    @JvmSynthetic
    internal fun startAutoWith(instance: Anvil) = this.also {
        initialTrajectory = instance.setPoseEstimateNow(instance.startPose).build()
        initialInstance = instance
    }
}
