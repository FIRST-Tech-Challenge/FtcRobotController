package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

public abstract class testAutoNoOdo extends LinearOpMode {
    static final double     COUNTS_PER_MOTOR_REV    = 767.2 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_CM       = 10.0 ;     // This measurement is more exact than inches
    static final double     COUNTS_PER_CM         = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * Math.PI));

    HardwareMapV2 robot = new HardwareMapV2();

    public void initHardware(){
        robot = new HardwareMapV2();
        robot.init(hardwareMap);


    }
}
