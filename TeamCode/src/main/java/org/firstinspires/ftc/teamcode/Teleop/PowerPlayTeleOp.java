package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.GBrobot;

@TeleOp
public class PowerPlayTeleOp extends OpMode {
    public GBrobot robot;

    @Override
    public void init() {
        this.robot = new GBrobot(this);
    }

    @Override
    public void loop() {

        // Claw controls
        if(gamepad2.dpad_up){
            robot.claw.setClawPosition(0.25);
        }

        if(gamepad2.dpad_down){
            robot.claw.setClawPosition(0);
        }

        // Drivetrain
        double drive = gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x * 1.1;
        double turn = -gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(drive)+Math.abs(strafe)+Math.abs(turn),1);

        double fl = ((drive+strafe+turn)/denominator);
        double bl = ((drive-strafe+turn)/denominator);
        double fr = ((drive-strafe-turn)/denominator);
        double br = ((drive+strafe-turn)/denominator);
        robot.Drive.setMotorPower(fl, bl, fr, br);

        // DR4B lift
        double lift = gamepad2.left_stick_y;
        robot.lift.SetMotorPower(lift);

    }
}
