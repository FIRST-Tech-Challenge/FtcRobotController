package com.technototes.library.measurement.ratio;

import com.technototes.library.measurement.unit.Unit;
@Deprecated
public abstract class Ratio<T extends Unit, U extends Unit> {
    public Ratio connectedRatio = null;
    public Ratio(T from, U to){

    }
    public Ratio to(Ratio r){
        connectedRatio = r;
        return connectedRatio;
    }
}
