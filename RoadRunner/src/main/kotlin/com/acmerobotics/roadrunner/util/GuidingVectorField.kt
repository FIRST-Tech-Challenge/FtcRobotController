package com.acmerobotics.roadrunner.util

import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.path.Path
import kotlin.math.sign

/**
 * Guiding vector field for effective path following described in section III, eq. (9) of
 * [1610.04391.pdf](https://arxiv.org/pdf/1610.04391.pdf). Implementation note: 2D parametric curves are used to
 * describe paths instead of implicit curves of the form f(x,y) = 0 as described in the paper (which dramatically
 * affects the cross track error calculation).
 *
 * @param path path to follow (interpolator is ignored)
 * @param kN path normal weight (see eq. (9))
 * @param errorMapFunc custom error mapping (see eq. (4))
 */
class GuidingVectorField(
    private val path: Path,
    private val kN: Double,
    private val errorMapFunc: (Double) -> Double = { it }
) {

    /**
     * Container for the direction of the GVF and intermediate values used in its computation.
     *
     * @param vector normalized direction vector of the GVF
     * @param pathPoint point on the path from the projection
     * @param displacement displacement along the path of [pathPoint]
     * @param error signed cross track error (distance between [pathPoint] and the query point)
     */
    data class GVFResult(
        val vector: Vector2d,
        val pathPoint: Vector2d,
        val displacement: Double,
        val error: Double
    )

    /**
     * Returns the normalized value of the vector field at the given point along with useful intermediate computations.
     */
    fun getExtended(x: Double, y: Double, projectGuess: Double = path.length() / 2.0): GVFResult {
        val point = Vector2d(x, y)
        val displacement = path.fastProject(Vector2d(x, y), projectGuess)
        val pathPoint = path[displacement].vec()
        val tangent = path.deriv(displacement).vec()
        val pathToPoint = point - pathPoint
        val orientation = -sign(pathToPoint.x * tangent.y - pathToPoint.y * tangent.x)
        val error = orientation * pathToPoint.norm()
        val normal = tangent.rotated(Math.PI / 2.0)
        val vector = tangent - normal * kN * errorMapFunc(error)
        return GVFResult(
            vector / vector.norm(),
            pathPoint,
            displacement,
            error
        )
    }

    /**
     * Returns the normalized value of the vector field at the given point.
     */
    operator fun get(x: Double, y: Double) = getExtended(x, y).vector
}
