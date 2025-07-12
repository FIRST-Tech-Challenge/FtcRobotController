package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.gb.pinpoint.driver.GoBildaPinpointDriver;

public class Robot {
    Drive_base db = new Drive_base();
    Arm arm = new Arm();
    LinLifts ll = new LinLifts();

    void init(HardwareMap hwMap) {
        db.init(hwMap);
        arm.init(hwMap);
//        ll.init(hwMap);
    }
}
