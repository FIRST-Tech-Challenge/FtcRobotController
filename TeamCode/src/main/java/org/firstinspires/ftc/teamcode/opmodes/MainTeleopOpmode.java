package org.firstinspires.ftc.teamcode.opmodes  ;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

@TeleOp
public class MainTeleopOpmode extends OpMode {

//    RobotHardware robot = new RobotHardware();
    DriveSubsystem drive_subsystem = new DriveSubsystem();


    @Override
    public void init() {
        drive_subsystem.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {
        telemetry.addData("Status", "OpMode is starting");
    }


    @Override
    public void loop() {
        telemetry.addData("gamepad 1 left stick y", gamepad1.left_stick_y);
        telemetry.addData("gamepad 1 right stick x", gamepad1.right_stick_x);
        drive_subsystem.Drive(gamepad1.left_stick_y, gamepad1.right_stick_x);

    }
}
