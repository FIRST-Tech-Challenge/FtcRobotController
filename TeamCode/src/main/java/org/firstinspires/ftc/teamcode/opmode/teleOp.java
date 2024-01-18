package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotContainer;

@TeleOp(name = "teleOp")
public class teleOp extends OpMode {
    Telemetry telemetry;
    Gamepad gamepad1;
    Gamepad gamepad2;
    //find a boolean supplier array of buttons
    RobotContainer robot = new RobotContainer(hardwareMap, telemetry, gamepad1, gamepad2);
    AnalogInput input;
    @Override
    public void init() {
        robot.bindCommands();
         input = hardwareMap.analogInput.get("p");
    }

    @Override
    public void loop() {
        robot.run();
        telemetry.addData("input",input.getVoltage());
    }
}
