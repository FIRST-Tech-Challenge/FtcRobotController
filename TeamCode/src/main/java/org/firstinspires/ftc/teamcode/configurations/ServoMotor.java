/* -------------------------------------------------------
   Copyright (c) [2024] FASNY
   All rights reserved
   -------------------------------------------------------
   Servo configuration
   ------------------------------------------------------- */

package org.firstinspires.ftc.teamcode.configurations;


public class ServoMotor {

    private String  m_name          = "";

    public ServoMotor(String Name) {
        m_name = Name;
    }

    public void setName(String Name) { m_name = Name;}

    public String getName()          { return m_name;}

}