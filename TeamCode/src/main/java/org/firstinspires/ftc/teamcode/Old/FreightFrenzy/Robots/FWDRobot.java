package org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Robots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robots.BasicRobot;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFMotor;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;

public class FWDRobot extends BasicRobot {
    public RFMotor motorLeftBack;
    public RFMotor motorRightBack;
    public RFMotor motorLeftFront;
    public RFMotor motorRightFront;
    private RFMotor wobbleArmGoal;
    private RFServo wobbleGoalGrabbyer;
    public FWDRobot(LinearOpMode op, boolean isTeleOp){
        super(op, isTeleOp);
        motorLeftBack = new RFMotor("motorLeftBack", DcMotor.RunMode.RUN_WITHOUT_ENCODER, false);
        motorRightBack = new RFMotor("motorRightBack", DcMotor.RunMode.RUN_WITHOUT_ENCODER, false);
        motorLeftFront = new RFMotor("motorLeftFront", DcMotor.RunMode.RUN_WITHOUT_ENCODER, false);
        motorRightFront = new RFMotor("motorRightFront", DcMotor.RunMode.RUN_WITHOUT_ENCODER, false);
        //wobbleArmGoal = new RFMotor("wobbleArmGoal", oop, DcMotor.RunMode.RUN_USING_ENCODER, false);
    }
}
