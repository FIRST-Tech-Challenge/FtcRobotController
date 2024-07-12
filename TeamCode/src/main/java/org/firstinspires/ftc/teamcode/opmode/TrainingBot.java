package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mechanism.Robot2Board;
import org.firstinspires.ftc.teamcode.mechanism.Traction;

@TeleOp
public class TrainingBot extends OpMode {
    Robot2Board Bored = new Robot2Board();
    Traction Grip = new Traction();
    double axial, lateral, yaw;
    @Override
    public void init(){
        Bored.init(hardwareMap);
    }
    @Override
    public void loop(){
        axial = gamepad1.left_stick_y;
        lateral = -gamepad1.left_stick_x;
        yaw = -gamepad1.right_stick_x;

        Grip.controllerDrive(axial, lateral, yaw);
    }
}
