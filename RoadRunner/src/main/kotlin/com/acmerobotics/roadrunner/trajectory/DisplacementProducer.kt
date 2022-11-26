package com.acmerobotics.roadrunner.trajectory

/**
 * SAM interface that produces a marker absolute displacement offset from the trajectory length.
 */
fun interface DisplacementProducer {
    fun produce(length: Double): Double
}
