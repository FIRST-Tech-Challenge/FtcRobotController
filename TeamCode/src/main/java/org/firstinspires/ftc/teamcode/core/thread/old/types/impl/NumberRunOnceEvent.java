package org.firstinspires.ftc.teamcode.core.thread.old.types.impl;

import org.firstinspires.ftc.teamcode.core.thread.old.types.api.RunListenerOnceEvent;

import java.util.function.Function;

/**
 * Basic event that compares a number. Static functions define lambda-free ways of creating an
 * instance of this class. YOU MUST USE ATOMICS, AND YOU HAVE TO USE .SET ON THE ATOMICS IN YOUR
 * OTHER CODE!!!
 */
public class NumberRunOnceEvent extends RunListenerOnceEvent {
    private final Number number;
    private final Function<Number, Boolean> logic;

    /**
     * @param number The number to run the function on. THIS MUST BE AN ATOMIC. {@link java.util.concurrent.atomic}
     * @param logic The function to apply.
     * @param listener The listener to be run.
     */
    public NumberRunOnceEvent(Number number, Function<Number, Boolean> logic, Runnable listener) {
        super(listener);
        this.number = number;
        this.logic = logic;
    }
    
    @Override
    public boolean shouldRun() {
        return logic.apply(number);
    }

    /**
     * Event that runs when the integer (byte, short, int, long) is equal to the number to compare to.
     * @param number Number to check. THIS MUST BE AN ATOMIC. {@link java.util.concurrent.atomic}
     * @param compareTo Number to compare to.
     * @param listener Listener to run.
     */
    public static NumberRunOnceEvent createEqualToInteger(Number number, long compareTo, Runnable listener) {
        return new NumberRunOnceEvent(number, (number1) -> number1.longValue() == compareTo, listener);
    }

    /**
     * Event that runs when the integer (byte, short, int, long) is not equal to the number to compare to.
     * @param number Number to check. THIS MUST BE AN ATOMIC. {@link java.util.concurrent.atomic}
     * @param compareTo Number to compare to.
     * @param listener Listener to run.
     */
    public static NumberRunOnceEvent createNotEqualToInteger(Number number, long compareTo, Runnable listener) {
        return new NumberRunOnceEvent(number, (number1) -> number1.longValue() != compareTo, listener);
    }

    /**
     * Event that runs when the integer (byte, short, int, long) is less than the number to compare to.
     * @param number Number to check. THIS MUST BE AN ATOMIC. {@link java.util.concurrent.atomic}
     * @param compareTo Number to compare to.
     * @param listener Listener to run.
     */
    public static NumberRunOnceEvent createLessThanInteger(Number number, long compareTo, Runnable listener) {
        return new NumberRunOnceEvent(number, (number1) -> number1.longValue() < compareTo, listener);
    }

    /**
     * Event that runs when the integer (byte, short, int, long) is greater than the number to compare to.
     * @param number Number to check. THIS MUST BE AN ATOMIC. {@link java.util.concurrent.atomic}
     * @param compareTo Number to compare to.
     * @param listener Listener to run.
     */
    public static NumberRunOnceEvent createGreaterThanInteger(Number number, long compareTo, Runnable listener) {
        return new NumberRunOnceEvent(number, (number1) -> number1.longValue() > compareTo, listener);
    }

    /**
     * Event that runs when the integer (byte, short, int, long) is less than or equal to the number to compare to.
     * @param number Number to check. THIS MUST BE AN ATOMIC. {@link java.util.concurrent.atomic}
     * @param compareTo Number to compare to.
     * @param listener Listener to run.
     */
    public static NumberRunOnceEvent createLessThanOrEqualToInteger(Number number, long compareTo, Runnable listener) {
        return new NumberRunOnceEvent(number, (number1) -> number1.longValue() <= compareTo, listener);
    }

    /**
     * Event that runs when the integer (byte, short, int, long) is greater than or equal to the number to compare to.
     * @param number Number to check. THIS MUST BE AN ATOMIC. {@link java.util.concurrent.atomic}
     * @param compareTo Number to compare to.
     * @param listener Listener to run.
     */
    public static NumberRunOnceEvent createGreaterThanOrEqualToInteger(Number number, long compareTo, Runnable listener) {
        return new NumberRunOnceEvent(number, (number1) -> number1.longValue() >= compareTo, listener);
    }
    
    /**
     * Event that runs when the floating point (float, double) is equal to the number to compare to.
     * @param number Number to check. THIS MUST BE AN ATOMIC. {@link java.util.concurrent.atomic}
     * @param compareTo Number to compare to.
     * @param listener Listener to run.
     */
    public static NumberRunOnceEvent createEqualToFloating(Number number, double compareTo, Runnable listener) {
        return new NumberRunOnceEvent(number, (number1) -> number1.doubleValue() == compareTo, listener);
    }

    /**
     * Event that runs when the floating point (float, double) is not equal to the number to compare to.
     * @param number Number to check. THIS MUST BE AN ATOMIC. {@link java.util.concurrent.atomic}
     * @param compareTo Number to compare to.
     * @param listener Listener to run.
     */
    public static NumberRunOnceEvent createNotEqualToFloating(Number number, double compareTo, Runnable listener) {
        return new NumberRunOnceEvent(number, (number1) -> number1.doubleValue() != compareTo, listener);
    }

    /**
     * Event that runs when the floating point (float, double) is less than the number to compare to.
     * @param number Number to check. THIS MUST BE AN ATOMIC. {@link java.util.concurrent.atomic}
     * @param compareTo Number to compare to.
     * @param listener Listener to run.
     */
    public static NumberRunOnceEvent createLessThanFloating(Number number, double compareTo, Runnable listener) {
        return new NumberRunOnceEvent(number, (number1) -> number1.doubleValue() < compareTo, listener);
    }

    /**
     * Event that runs when the floating point (float, double) is greater than to the number to compare to.
     * @param number Number to check. THIS MUST BE AN ATOMIC AND A FLOATING POINT NUMBER. {@link java.util.concurrent.atomic}
     * @param compareTo Number to compare to.
     * @param listener Listener to run.
     */
    public static NumberRunOnceEvent createGreaterThanFloating(Number number, double compareTo, Runnable listener) {
        return new NumberRunOnceEvent(number, (number1) -> number1.doubleValue() > compareTo, listener);
    }

    /**
     * Event that runs when the floating point (float, double) is less than or equal to the number to compare to.
     * @param number Number to check. THIS MUST BE AN ATOMIC AND A FLOATING POINT NUMBER. {@link java.util.concurrent.atomic}
     * @param compareTo Number to compare to.
     * @param listener Listener to run.
     */
    public static NumberRunOnceEvent createLessThanOrEqualToFloating(Number number, double compareTo, Runnable listener) {
        return new NumberRunOnceEvent(number, (number1) -> number1.doubleValue() <= compareTo, listener);
    }

    /**
     * Event that runs when the floating point (float, double) is greater than equal to the number to compare to.
     * @param number Number to check. THIS MUST BE AN ATOMIC AND A FLOATING POINT NUMBER. {@link java.util.concurrent.atomic}
     * @param compareTo Number to compare to.
     * @param listener Listener to run.
     */
    public static NumberRunOnceEvent createGreaterThanOrEqualToFloating(Number number, double compareTo, Runnable listener) {
        return new NumberRunOnceEvent(number, (number1) -> number1.doubleValue() >= compareTo, listener);
    }
}
