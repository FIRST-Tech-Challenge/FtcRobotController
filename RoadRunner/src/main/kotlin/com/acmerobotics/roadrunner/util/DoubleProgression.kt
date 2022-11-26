package com.acmerobotics.roadrunner.util

import kotlin.math.ceil
import kotlin.math.floor

/**
 * A progression of values of type `Double`.
 */
data class DoubleProgression(
    val start: Double,
    val step: Double,
    val size: Int
) : Iterable<Double> {
    companion object {
        @JvmStatic
        fun fromClosedInterval(start: Double, endInclusive: Double, count: Int): DoubleProgression {
            val step = when (count) {
                0 -> 0.0
                1 -> 1.0
                else -> (endInclusive - start) / (count - 1)
            }
            return DoubleProgression(start, step, count)
        }
    }

    operator fun plus(offset: Double) =
        DoubleProgression(start + offset, step, size)

    operator fun minus(offset: Double) =
        DoubleProgression(start - offset, step, size)

    operator fun unaryMinus() = DoubleProgression(-start, -step, size)

    fun isEmpty() = size == 0

    private fun rawIndex(query: Double) = (query - start) / step

    fun floorIndex(query: Double) = floor(rawIndex(query)).toInt()

    fun ceilIndex(query: Double) = ceil(rawIndex(query)).toInt()

    operator fun get(index: Int) = start + step * index

    operator fun contains(query: Double): Boolean {
        val rawIndex = rawIndex(query)
        return if (rawIndex < 0) {
            false
        } else {
            ceil(rawIndex) < size
        }
    }

    fun size() = size

    fun split(sep: Double): Pair<DoubleProgression, DoubleProgression> {
        val sepIndex = ceilIndex(sep)
        return when {
            sepIndex < 0 -> DoubleProgression(sep, step, 0) to this
            sepIndex >= size -> this to DoubleProgression(sep, step, 0)
            else -> DoubleProgression(start, step, sepIndex) to
                DoubleProgression(get(sepIndex), step, size - sepIndex)
        }
    }

    /**
     * Iterator implementation for [DoubleProgression].
     */
    @Suppress("IteratorNotThrowingNoSuchElementException")
    inner class IteratorImpl : Iterator<Double> {
        private val iterator: Iterator<Int> = IntRange(0, size - 1).iterator()

        override fun hasNext() = iterator.hasNext()

        override fun next() = get(iterator.next())
    }

    override fun iterator() = IteratorImpl()
}

operator fun Double.plus(progression: DoubleProgression) = progression + this

operator fun Double.minus(progression: DoubleProgression) =
    DoubleProgression(this - progression.start, -progression.step, progression.size)
