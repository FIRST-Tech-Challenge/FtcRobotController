package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.ImprovedGamepad;
import org.firstinspires.ftc.teamcode.hardware.UpdatedClawbotHardware;

@TeleOp(name="Firstname Lastname first auto", group="Outreach")
public class ExampleTeleop extends OpMode {
    UpdatedClawbotHardware robot = new UpdatedClawbotHardware();
    ImprovedGamepad gamepad;
    int state = 0;
    double rightStickValue = 0;

    @Override
    public void init() {
        gamepad = new ImprovedGamepad(gamepad1, new ElapsedTime(), "Gamepad");
        robot.init(this.hardwareMap);
    }

    @Override
    public void loop() {
        gamepad.update();
        //write all teleop code below here
        robot.leftDrive.setPower(gamepad.left_stick_x.getValue() * -1);
    }
}
