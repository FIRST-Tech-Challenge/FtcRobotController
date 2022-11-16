package org.firstinspires.ftc.teamcode.Test;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.GBrobot;
@TeleOp
public class LimitSwitchTest extends OpMode {
    public GBrobot robot;
    @Override
    public void init() {
        robot = new GBrobot(this);
    }

    @Override
    public void loop() {
        robot.lift.liftMotor1.setPower(1);
        if(robot.limit1.isPressed()){
            telemetry.addData("IS PRESSED:" , " true");
            robot.lift.liftMotor1.setPower(0);
        }
        else{
            telemetry.addData("IS PRESSED", "false");
        }
        telemetry.update();
    }
}
