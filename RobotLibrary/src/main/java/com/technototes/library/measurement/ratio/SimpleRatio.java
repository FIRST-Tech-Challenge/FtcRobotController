package com.technototes.library.measurement.ratio;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

/** Class for simple ratios
 * @author Alex Stedman
 */
public class SimpleRatio {
    /** List for chained ratios
     *
     */
    public List<SimpleRatio> chainedRatios;
    /** The ratio constant for the single ratio
     *
     */
    public double unit;

    /** Create a simple ratio
     *
     * @param units The units to make the ratio from
     */
    public SimpleRatio(double... units){
        this(units[0], units[1]);
        for(int i = 1; i < units.length-1; i++) {
            add(units[i], units[i+1]);
        }
    }

    /** Create a simple ratio
     *
     * @param num The ratio number
     */
    public SimpleRatio(double num){
        chainedRatios = new ArrayList<>();
        unit = num;
    }
    /** Create a simple ratio where the two numbers are divided by each other
     *
     * @param num1 The input for the ratio
     * @param num2 The output for the ratio
     *
     */
    public SimpleRatio(double num1, double num2){
        this(num1/num2);
    }

    /** Add a ratio to the current ratio for chaining
     *
     * @param ratio The new ratio
     * @return this
     */
    public SimpleRatio add(SimpleRatio ratio){
        return add(ratio.get());
    }
    /** Add a ratio to the current ratio for chaining
     *
     * @param num The number to make the new ratio with
     * @return this
     */
    public SimpleRatio add(double num){
        chainedRatios.add(new SimpleRatio(num));
        return this;
    }
    /** Add a ratio to the current ratio for chaining
     *
     * @param num1 The input number to make the new ratio with
     * @param num2 The output number to make the new ratio with
     * @return this
     */
    public SimpleRatio add(double num1, double num2){
        return add(num1/num2);
    }

    /** Returns the ratio value
     *
     * @return The ratio for input 1
     */
    public double get(){
        return get(1);
    }
    /** Returns the ratio value
     * @param num The input for the ratio
     * @return The ratio for input num
     */
    public double get(double num){
        AtomicReference<Double> ret = new AtomicReference<>(num * unit);
        chainedRatios.forEach((simpleRatio -> ret.set(simpleRatio.get(ret.get()))));
        return ret.get();
    }

    @Override
    public String toString() {
        return get()+"";
    }
}
