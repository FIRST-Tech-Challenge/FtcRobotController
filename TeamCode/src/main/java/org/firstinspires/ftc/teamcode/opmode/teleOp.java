package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotContainer;

@TeleOp
public class teleOp extends OpMode {
    RobotContainer robot;
    AnalogInput pot;
    Gamepad gamepad = new Gamepad();

    @Override
    public void init() {
        robot=new RobotContainer(hardwareMap);
        robot.bindCommands(gamepad);
        pot=hardwareMap.analogInput.get("pot");

    }
    @Override
    public void loop() {
        robot.run();
        telemetry.addData("port0", pot.getVoltage());
    }
}
