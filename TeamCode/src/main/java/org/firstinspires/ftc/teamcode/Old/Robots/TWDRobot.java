package org.firstinspires.ftc.teamcode.Old.Robots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robots.BasicRobot;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFMotor;

public class TWDRobot extends BasicRobot {
    public RFMotor motorLeft;
    public RFMotor motorRight;
    public TWDRobot(LinearOpMode op, boolean isTeleOp){
        super(op, isTeleOp);
        motorLeft = new RFMotor("motorLeft", DcMotor.RunMode.RUN_WITHOUT_ENCODER, false);
        motorRight = new RFMotor("motorRight", DcMotor.RunMode.RUN_WITHOUT_ENCODER, false);
    }
}
