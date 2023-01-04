@file:Suppress("HasPlatformType")

package ftc.rogue.blacksmith.meepmeep

import java.util.concurrent.Executors

object ScheduledMeepMeepExecutor {
    val EXECUTOR = Executors.newSingleThreadScheduledExecutor()
}
