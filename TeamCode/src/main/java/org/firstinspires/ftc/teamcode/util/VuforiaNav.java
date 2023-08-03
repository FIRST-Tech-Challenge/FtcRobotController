package org.firstinspires.ftc.teamcode.util;

public class VuforiaNav
{

    //////////////////////////
    ////// Singleton Functions

    private static VuforiaNav m_VuforiaNav;

    private VuforiaNav()
    {

    }

    public static VuforiaNav getInstance()
    {
        if (m_VuforiaNav == null) m_VuforiaNav = new VuforiaNav();

        return m_VuforiaNav;
    }

    ////// Singleton Functions
    //////////////////////////

    private static class MyThread extends Thread
    {
        public void run()
        {

        }
    }

}


