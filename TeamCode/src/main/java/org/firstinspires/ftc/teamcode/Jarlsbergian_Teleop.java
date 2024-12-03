package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "YARRRRRG")
public class Jarlsbergian_Teleop extends OpMode {

    JarlsCHasse drivetrain;
    Odometry_Info odo;
    CheetoFingers arm;

    @Override
    public void init() {
        drivetrain = new JarlsCHasse(hardwareMap);
        odo = new Odometry_Info(hardwareMap);
        arm = new CheetoFingers(hardwareMap);
        odo.resetEncoders();
    }

    @Override
    public void loop() {
        drivetrain.GamepadInputs(odo.cur0, gamepad1);
        arm.gamepadInputsArm(gamepad1);
        odo.updateCurPos();

        telemetry.addData("Rotation", Math.toDegrees(odo.cur0));
        telemetry.addData("X Value", odo.Xc);
        telemetry.addData("Y Value", odo.Xp);
        telemetry.addData("odoback", odo.Cn3);

        telemetry.addData("Right Finger", arm.FINGER_RS.getPosition());
        telemetry.addData("Left Finger", arm.FINGER_LS.getPosition());


    }
}
