package ftc.rogue.blacksmith.internal.anvil

import com.acmerobotics.roadrunner.geometry.Pose2d
import ftc.rogue.blacksmith.internal.AnvilRunConfigBuilder

class AnvilRunConfig internal constructor() {
    @get:JvmSynthetic internal var buildsSynchronously = false
        private set

    @get:JvmSynthetic internal var runsSynchronously = false
        private set

    @get:JvmSynthetic internal var predicate = { true }
        private set

    @get:JvmSynthetic internal var startPoseSupplier: (() -> Pose2d)? = null
        private set

    @JvmOverloads
    fun buildSynchronously(yes: Boolean = true): AnvilRunConfig {
        buildsSynchronously = yes
        return this
    }

    @JvmOverloads
    fun runSynchronously(yes: Boolean = true): AnvilRunConfig {
        runsSynchronously = yes
        return this
    }

    fun runOnlyIf(condition: () -> Boolean): AnvilRunConfig {
        predicate = condition
        return this
    }

    fun setStartPose(supplier: () -> Pose2d): AnvilRunConfig {
        startPoseSupplier = supplier
        return this
    }

    companion object {
        @JvmField
        val DEFAULT = AnvilRunConfigBuilder { AnvilRunConfig() }
    }
}
