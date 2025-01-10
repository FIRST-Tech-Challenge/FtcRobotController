/* -------------------------------------------------------
   Copyright (c) [2024] FASNY
   All rights reserved
   -------------------------------------------------------
   IMU configuration
   ------------------------------------------------------- */

package org.firstinspires.ftc.teamcode.configurations;

/* Qualcomm includes */
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class ConfImu {

    private String                                          m_name = "";
    private RevHubOrientationOnRobot.LogoFacingDirection    m_logo = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    private RevHubOrientationOnRobot.UsbFacingDirection     m_usb  = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;

    public ConfImu(String Name, RevHubOrientationOnRobot.LogoFacingDirection Logo, RevHubOrientationOnRobot.UsbFacingDirection Usb) {
        m_name = Name;
        m_logo = Logo;
        m_usb  = Usb;
    }

    public ConfImu(String Name) {
        m_name = Name;
    }

    public void setName(String Name)                                       { m_name = Name;}
    public void setLogo(RevHubOrientationOnRobot.LogoFacingDirection Logo) { m_logo = Logo;}
    public void setUsb(RevHubOrientationOnRobot.UsbFacingDirection  Usb)   { m_usb = Usb;  }

    public String                                       getName()    { return m_name; }
    public RevHubOrientationOnRobot.LogoFacingDirection getLogo()    { return m_logo; }
    public RevHubOrientationOnRobot.UsbFacingDirection  getUsb()     { return m_usb;  }

}