/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   Servos configuration data
   ------------------------------------------------------- */

package org.firstinspires.ftc.teamcode.configurations;

/* System includes */
import java.util.LinkedHashMap;
import java.util.Map;

public class ConfServo {

    private       boolean                mShallMock   = false;

    private final Map<String, Boolean>   mHw          = new LinkedHashMap<>();
    private final Map<String, Double>    mPositions   = new LinkedHashMap<>();

    public ConfServo(String Name, boolean ShallReverse)
    {
        mHw.clear();
        mHw.put(Name,ShallReverse);
        mShallMock    = false;
    }

    public ConfServo(String Name1, boolean ShallReverse1, String Name2, boolean ShallReverse2)
    {
        mHw.clear();
        mHw.put(Name1,ShallReverse1);
        mHw.put(Name2,ShallReverse2);
        mShallMock    = false;
    }

    public void addHw(String Name, boolean ShallReverse) { mHw.put(Name,ShallReverse);  }
    public void addPosition(String Name, Double Value)   { mPositions.put(Name, Value); }

    public Map<String, Boolean> getHw()                  { return mHw;}
    public boolean              shallMock()              { return mShallMock; }
    public Map<String, Double>  getPositions()           { return mPositions; }
    public Double               getPosition(String Name) {
        Double result = -1.0;
        if(mPositions.containsKey(Name)) { result = mPositions.get(Name); }
        return result;
    }

}