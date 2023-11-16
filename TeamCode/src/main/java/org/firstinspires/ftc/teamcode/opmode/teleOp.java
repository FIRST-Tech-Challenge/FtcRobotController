package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.controller.PController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.util.SerialNumber;

import org.firstinspires.ftc.teamcode.RobotContainer;

@TeleOp(name = "teleOp")
public class teleOp extends OpMode {
    RobotContainer robot =new RobotContainer(hardwareMap);
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
