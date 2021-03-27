package com.technototes.logger;


import java.util.function.Supplier;

/** Class for returning things state to log at a higher level
 * @author Alex Stedman
 * @param <T> The type of object for the state
 */
public interface Stated<T> extends Supplier<T> {
    /** The method to get the state
     *
     * @return The state as T
     */
    T getState();

    @Override
    default T get(){
        return getState();
    }

}
