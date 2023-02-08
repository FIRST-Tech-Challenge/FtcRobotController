@file:Suppress("IllegalIdentifier")

package ftc.rogue.blacksmith.annotations

@Target(AnnotationTarget.CLASS, AnnotationTarget.FILE)
@Retention(AnnotationRetention.RUNTIME)
annotation class ConfigKt (
    val value: String = ""
)
