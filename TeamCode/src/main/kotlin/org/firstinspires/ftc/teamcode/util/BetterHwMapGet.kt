package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.HardwareMap

/**
 * Syntactic sugar for [HardwareMap.get].
 *
 * __Note: This is Kotlin only.__
 *
 * Basically, due to type erasure with Java generics, when using [HardwareMap.get], you need to
 * manually pass in the class of the component you want to get. e.g.
 * ```
 * val motor = hardwareMap.get(DcMotorEx::class.java, "motor")
 * ```
 * Using reified types, we can simply do this instead:
 * ```
 * val motor = hardwareMap._get<DcMotorEx>("motor")
 * //or
 * val motor: DcMotorEx = hardwareMap._get("motor")
 * ```
 *
 * _Ok in hindsight this doesn't appear as useful as I thought, but it's cooler and makes me feel
 * smart so shut up_
 *
 * @param T The type of the component you want to get.
 * @param name The name of the component you want to get.
 * @author KG
 */
private class KDocBugsOutForSingleExpressionFunctionsAndDoesntRenderSoJustIgnoreThisClassPlease
private class IJustPutTheseHereToMakeKDocsWorkIgnoreThemTheDocRefersToHardwareMapDotUnderscoreGet

inline fun <reified T> HardwareMap._get(name: String): T = get(T::class.java, name)