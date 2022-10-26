package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.GBrobot;

@TeleOp
public class testDR4B extends OpMode {
    GBrobot robot;

    @Override
    public void init() {
        robot = new GBrobot( this );

        telemetry.addLine( "Ready!" );
        telemetry.update( );
    }

    @Override
    public void loop() {
        telemetry.addLine("GamePad 2 Left JoyStick Y Controls the lift up or down " );
        telemetry.update( );

        double lift = gamepad2.left_stick_y;
        robot.lift.SetMotorPower(lift);
    }
}
