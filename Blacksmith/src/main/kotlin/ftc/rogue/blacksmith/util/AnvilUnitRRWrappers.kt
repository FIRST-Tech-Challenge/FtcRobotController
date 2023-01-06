@file:JvmName("YouShouldStaticImportTheMethodsInThisFile")
@file:JvmMultifileClass

package ftc.rogue.blacksmith.util

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d

fun bsmPose2d(x: Number, y: Number, heading: Number) = Pose2d(x.toIn(), y.toIn(), heading.toRad())
fun bsmPose2d(x: Double, y: Double, heading: Double) = Pose2d(x.toIn(), y.toIn(), heading.toRad())
fun bsmPose2d(x: Int, y: Int, heading: Int) = Pose2d(x.toIn(), y.toIn(), heading.toRad())

fun bsmVector2d(x: Number, y: Number) = Vector2d(x.toIn(), y.toIn())
fun bsmVector2d(x: Double, y: Double) = Vector2d(x.toIn(), y.toIn())
fun bsmVector2d(x: Int, y: Int) = Vector2d(x.toIn(), y.toIn())
