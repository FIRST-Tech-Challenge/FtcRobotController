package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Components.Logger;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFMotor;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;

public class FWDRobot extends BasicRobot{
    private RFMotor motorLeftBack;
    private RFMotor motorRightBack;
    private RFMotor motorLeftFront;
    private RFMotor motorRightFront;
    private RFMotor wobbleArmGoal;
    private RFServo wobbleGoalGrabbyer;
    public FWDRobot(LinearOpMode op){
        super(op);
        motorLeftBack = new RFMotor("motorLeftBack", DcMotor.RunMode.RUN_WITHOUT_ENCODER, false);
        motorRightBack = new RFMotor("motorRightBack", DcMotor.RunMode.RUN_WITHOUT_ENCODER, false);
        motorLeftFront = new RFMotor("motorLeftFront", DcMotor.RunMode.RUN_WITHOUT_ENCODER, false);
        motorRightFront = new RFMotor("motorRightFront", DcMotor.RunMode.RUN_WITHOUT_ENCODER, false);
        //wobbleArmGoal = new RFMotor("wobbleArmGoal", oop, DcMotor.RunMode.RUN_USING_ENCODER, false);
    }
}
