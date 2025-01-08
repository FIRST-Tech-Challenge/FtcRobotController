/* -------------------------------------------------------
   Copyright (c) [2024] FASNY
   All rights reserved
   -------------------------------------------------------
   Motor configuration
   ------------------------------------------------------- */

package org.firstinspires.ftc.teamcode.configurations;


public class Motor {

    private String  m_name          = "";
    private boolean m_shall_reverse = false;

    public Motor(String Name, boolean ShallReverse) {
        m_name = Name;
        m_shall_reverse = ShallReverse;
    }

    public void setName(String Name)             { m_name = Name;}
    public void setReverse(boolean ShallReverse) { m_shall_reverse = ShallReverse;}

    public String getName()        { return m_name;}
    public boolean getReverse()    { return m_shall_reverse;}

}