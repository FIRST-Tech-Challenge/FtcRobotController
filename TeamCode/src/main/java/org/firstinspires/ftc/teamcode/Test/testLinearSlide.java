package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Robot.GBrobot;

@TeleOp
public class testLinearSlide extends OpMode {
    GBrobot robot;

    @Override
    public void init() {
        robot = new GBrobot( this );

        telemetry.addLine( "Ready!" );
        telemetry.update( );
    }

    @Override
    public void loop() {
        /*telemetry.addLine("GamePad 2 Left JoyStick Y Controls the lift up or down " );
        telemetry.update( );
        robot.lift.SetMotorPower(gamepad2.left_stick_y*0.70);*/

        if (gamepad2.a) {
            robot.lift.driveLiftToPosition(0.7, 300);
        }
        if (gamepad2.y) {
            robot.lift.driveLiftToPosition(0.7, 500);
        }
        if (gamepad2.b) {
            robot.lift.driveLiftToPosition(0.7, 800);
        }
        if (gamepad2.x) {
            robot.lift.driveLiftToPosition(0.7, -300);
        }
        if (gamepad2.right_bumper) {
            robot.lift.SetMotorPower(0.0);
        }
    }

}
