package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.GBrobot;

@TeleOp
public class testIntake extends OpMode {
    GBrobot robot;

    @Override
    public void init() {
        robot = new GBrobot( this );
        telemetry.addLine( "Ready!" );
        telemetry.update( );
    }

    @Override
    public void loop() {
        telemetry.addLine("GamePad 2 A and B buttons Controls the claw" );
        telemetry.update( );

        if(gamepad2.a){
            robot.claw.setClawPosition(0.25);
        }

        if(gamepad2.b){
            robot.claw.setClawPosition(0);
        }
    }
}
