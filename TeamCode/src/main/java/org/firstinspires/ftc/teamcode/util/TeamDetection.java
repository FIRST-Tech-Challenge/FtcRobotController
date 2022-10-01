package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TeamDetection {
    public boolean redTeam = false;
    public boolean blueTeam = false;



    public TeamDetection(HardwareMap hardwareMap){
        DigitalChannel blueSwitch = hardwareMap.get(DigitalChannel.class, "blueSwitch");
        blueSwitch.setMode(DigitalChannel.Mode.INPUT);
         = blueSwitch.getState();
    }
}
