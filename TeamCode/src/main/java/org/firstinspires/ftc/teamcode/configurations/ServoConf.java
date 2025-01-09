/* -------------------------------------------------------
   Copyright (c) [2024] FASNY
   All rights reserved
   -------------------------------------------------------
   Servo configuration
   ------------------------------------------------------- */

package org.firstinspires.ftc.teamcode.configurations;

/* System includes */
import java.util.HashMap;
import java.util.Map;

public class ServoConf {

    private String              m_name          = "";
    private boolean             m_shall_reverse = false;
    private Map<String, Double> m_positions     = new HashMap<>();


    public ServoConf(String Name, boolean ShallReverse) {
        m_name = Name;
        m_shall_reverse = ShallReverse;
    }

    public void setName(String Name)                    { m_name = Name;}
    public void setReverse(boolean ShallReverse)        { m_shall_reverse = ShallReverse;}
    public void setPosition(String Name, Double Value)  { m_positions.put(Name, Value); }

    public String              getName()                { return m_name;}
    public boolean             getReverse()             { return m_shall_reverse;}
    public Map<String, Double> getPositions()           { return m_positions; }
    public Double              getPosition(String Name) {
        Double result = -1.0;
        if(m_positions.containsKey(Name)) { result = m_positions.get(Name); }
        return result;
    }

}