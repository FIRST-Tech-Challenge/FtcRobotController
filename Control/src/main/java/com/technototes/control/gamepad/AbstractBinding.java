package com.technototes.control.gamepad;

import java.util.Arrays;
import java.util.List;
import java.util.function.BooleanSupplier;

/** Class for bindings to extend
 * @author Alex Stedman
 * @param <T> The class for buttons
 */
public abstract class AbstractBinding<T extends GamepadButton> implements BooleanSupplier {
    /** Button type
     *
     */
    public enum Type {
        NONE_ACTIVE, SOME_ACTIVE, ALL_ACTIVE

    }
    private Type defaultType;
    private List<BooleanSupplier> suppliers;

    /** Make binding
     *
     * @param buttons The suppliers
     */
    public AbstractBinding(BooleanSupplier... buttons){
        this(Type.ALL_ACTIVE, buttons);
    }
    /** Make binding
     *
     * @param buttons The suppliers
     * @param type The button type
     */
    public AbstractBinding(Type type, BooleanSupplier... buttons){
        suppliers = Arrays.asList(buttons);
        defaultType = type;
    }

    @Override
    public boolean getAsBoolean() {
        return get(defaultType);
    }

    /** Get this as boolean for the type
     *
     * @param type The type to get boolean as
     * @return If the binding meets the criteria
     */
    public boolean get(Type type){
        boolean on=false, off=false;
        for(BooleanSupplier s : suppliers){
            if(s.getAsBoolean()){
                on=true;
            }else{
                off=true;
            }
        }
        switch (type){
            case NONE_ACTIVE:
                return !on;
            case ALL_ACTIVE:
                return !off;
            default:
                return on;

        }

    }

    /** Get this as button
     *
     * @return A button of default type
     */
    public T getAsButton(){
        return getAsButton(defaultType);
    }

    /** Get this as button
     *
     * @param type The binding type
     * @return A button of specified type
     */
    public abstract T getAsButton(Type type);

    /** Get the default type
     *
     * @return The default type
     */
    public Type getDefaultType(){
        return defaultType;
    }

    /** Set binding type
     *
     * @param type The type
     * @return this
     */
    public AbstractBinding setType(Type type){
        defaultType = type;
        return this;
    }

}
