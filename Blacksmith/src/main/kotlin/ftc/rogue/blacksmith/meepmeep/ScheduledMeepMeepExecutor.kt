@file:Suppress("HasPlatformType")

package ftc.rogue.blacksmith.meepmeep

import java.util.concurrent.Executors

internal object ScheduledMeepMeepExecutor {
    val EXECUTOR = Executors.newSingleThreadScheduledExecutor()
}
