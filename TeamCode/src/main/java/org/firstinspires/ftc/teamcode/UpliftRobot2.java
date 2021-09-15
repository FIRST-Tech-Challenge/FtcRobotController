package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class UpliftRobot2 {
    public DcMotor tm;
    public LinearOpMode opMode;
    public HardwareMap hardwareMap;


    // robot constructor
    public UpliftRobot2(LinearOpMode opMode) {
        this.opMode = opMode;
        getHardware();
    }

    public void getHardware() {
        hardwareMap = opMode.hardwareMap;
        tm = hardwareMap.get(DcMotor.class, "test_motor");


    }
}
