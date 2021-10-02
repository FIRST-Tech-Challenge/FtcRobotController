package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class robotHardware {

    HardwareMap hMap;
    public YellowJacket19_2 leftFront;
    public YellowJacket19_2 rightFront;
    public YellowJacket19_2 leftBack;
    public YellowJacket19_2 rightBack;
    RevIMU revIMU;

    public robotHardware() {

    }

    public void init(HardwareMap hMap) {
        leftFront = new YellowJacket19_2(hMap, "LF");
        rightFront = new YellowJacket19_2(hMap, "RF");
        leftBack = new YellowJacket19_2(hMap, "LB");
        rightBack = new YellowJacket19_2(hMap, "RB");
    }

}
