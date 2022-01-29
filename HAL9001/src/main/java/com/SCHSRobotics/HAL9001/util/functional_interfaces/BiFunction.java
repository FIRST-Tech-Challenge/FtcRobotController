package com.SCHSRobotics.HAL9001.util.functional_interfaces;

/**
 * An arbitrary function with 2 inputs and 1 output.
 * <p>
 * Creation Date: 7/18/19
 *
 * @param <T> The datatype of the first input.
 * @param <R> The datatype of the second input.
 * @param <S> The datatype of the output.
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @since 1.0.0
 */
@FunctionalInterface
public interface BiFunction<T, R, S> {

    /**
     * The function that gets overridden when implementing BiFunction (usually via lambda).
     *
     * @param arg1 The first parameter.
     * @param arg2 The second parameter.
     * @return The output.
     */
    S apply(T arg1, R arg2);
}
