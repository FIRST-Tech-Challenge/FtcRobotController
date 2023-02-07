@file:Suppress("HasPlatformType")

package ftc.rogue.blacksmith.util.meepmeep

import java.util.concurrent.Executors

internal object ScheduledMeepMeepExecutor {
    @get:JvmSynthetic
    val EXECUTOR = Executors.newSingleThreadScheduledExecutor()
}
