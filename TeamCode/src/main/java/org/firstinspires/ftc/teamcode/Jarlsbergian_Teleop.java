package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Jarlsbergian_Teleop extends OpMode {

    JarlsCHasse drivetrain = null;
    Odometry_Info odo = null;


    @Override
    public void init() {
        JarlsCHasse driveTrain = new JarlsCHasse(hardwareMap);
        Odometry_Info odo = new Odometry_Info(hardwareMap);
    }

    @Override
    public void loop() {

        odo.updateCurPos();
        drivetrain.GamepadInputs(0, gamepad1);

    }
}
