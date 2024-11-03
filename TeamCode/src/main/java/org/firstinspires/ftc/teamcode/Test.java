package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Sensors.Gyro;
import org.firstinspires.ftc.teamcode.Utils.Vector;
import org.firstinspires.ftc.teamcode.robotSubSystems.Drivetrain.Drivetrain;
@TeleOp(name = "Test")
public class Test extends OpMode {
    @Override
    public void init() {
        Drivetrain.init(hardwareMap);
        Gyro.init(hardwareMap);
    }
    @Override
    public void loop() {
        Drivetrain.operate(new Vector(gamepad1.left_stick_x, -gamepad1.left_stick_y), gamepad1.right_trigger - gamepad1.left_trigger);
        if (gamepad1.back) {Gyro.resetGyro();};
        telemetry.addData("Gyro", Gyro.getAngle());
    }
}
