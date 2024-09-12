package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {


    HardwareMap hardwareMap;
    MecanumDriveTrain driveTrain = new MecanumDriveTrain();
    LinearVerticalLeft linear_L = new LinearVerticalLeft();
    LinearVerticalRight linear_R = new LinearVerticalRight();



    public void init(HardwareMap ahwMap) {
        hardwareMap = ahwMap;
        driveTrain.init(hardwareMap);
        linear_L.init(hardwareMap);
        linear_R.init(hardwareMap);

    }

}
