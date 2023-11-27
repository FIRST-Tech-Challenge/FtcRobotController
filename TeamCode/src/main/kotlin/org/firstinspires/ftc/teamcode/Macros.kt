package org.firstinspires.ftc.teamcode

/**
 * **I Don't Care** if it's `null`, but I don't want an exception, so I use the `idc` function
 */
inline infix fun <T, I> I.idc(f: () -> T): T? = try { f() } catch (_: Exception) { null }

fun Double.clamp(min: Double, max: Double) = if (this < min) min; else if (this > max) max; else this