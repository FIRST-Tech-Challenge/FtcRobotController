package org.firstinspires.ftc.teamcode.Team.BasicRobots

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap

class Holonomic {
    var RFMotor: DcMotor? = null
    var RBMotor: DcMotor? = null
    var LFMotor: DcMotor? = null
    var LBMotor: DcMotor? = null

    fun init(Map: HardwareMap) {
        LFMotor = Map.dcMotor["FLDrive"]
        LBMotor = Map.dcMotor["BLDrive"]
        RFMotor = Map.dcMotor["FRDrive"]
        RBMotor = Map.dcMotor["BRDrive"]

        LFMotor?.power = 0.0
        LBMotor?.power = 0.0
        RFMotor?.power = 0.0
        RBMotor?.power = 0.0

        LFMotor?.mode = DcMotor.RunMode.RUN_USING_ENCODER
        RFMotor?.mode = DcMotor.RunMode.RUN_USING_ENCODER
        LBMotor?.mode = DcMotor.RunMode.RUN_USING_ENCODER
        RBMotor?.mode = DcMotor.RunMode.RUN_USING_ENCODER

        RFMotor?.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        RBMotor?.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        LFMotor?.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        LBMotor?.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }
}