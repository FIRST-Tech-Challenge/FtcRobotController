package com.technototes.logger.entry;


import com.technototes.logger.Color;

import java.util.function.Supplier;

/** The root class for logging entries
 * @author Alex Stedman
 *
 * @param <T> The type of value being stored by the entry
 */
public abstract class Entry<T> implements Supplier<T> {

    protected int x;
    protected int priority;
    protected Supplier<T> supplier;
    protected String name;
    protected String tag;
    protected Color color;

    public Entry(String n, Supplier<T> s, int index, Color c){
        x = index;
        supplier = s;
        name = n;
        color = c;
        tag =  (name.equals("") ? " " : color.format(name)+" ` ");
    }
    public Entry<T> setPriority(int p){
        priority = p;
        return this;
    }
    @Override
    public T get() {
        return supplier.get();
    }

    /** The String for the logged item
     *
     * @return The String
     */
    @Override
    public String toString() {
        return supplier.get().toString();
    }

    /** The tag for the entry
     *
     * @return The tag
     */
    public String getTag(){
        return tag;
    }

    /** Get the name (unformatted tag)
     *
     * @return The name
     */
    public String getName() {
        return name;
    }

    /** Get the index for the entry
     *
     * @return The index
     */
    public int getIndex() {
        return x;
    }

    /** Set index
     *
     * @param i New index
     * @return this
     */
    public Entry setIndex(int i){
        x = i;
        return this;
    }

    /** Get Priority for the entry
     * @return The priority
     */
    public int getPriority(){
        return priority;
    }


}
