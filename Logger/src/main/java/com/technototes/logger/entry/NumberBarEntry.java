package com.technototes.logger.entry;

import java.util.function.Supplier;

public class NumberBarEntry extends NumberSliderEntry {

    public NumberBarEntry(String n, Supplier<Number> s, int x, Number mi, Number ma, Number sc) {
        super(n, s, x, mi, ma, sc);
    }

    @Override
    public String toString() {
        String r = name+": [";
        for(double i = (double)min; i <= (double)max; i+=(double)scale) {
            if(Math.abs((double)get()-i)*2 < (double)scale){
                r+="#";
            }else if((double)get() < i){
                r+="~";
            }else{
                r+="=";
            }
        }
        return r+"]";
    }

}
