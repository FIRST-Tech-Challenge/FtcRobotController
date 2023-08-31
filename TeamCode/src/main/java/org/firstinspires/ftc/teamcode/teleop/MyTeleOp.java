package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.command.CameraStream;
import org.firstinspires.ftc.teamcode.command.FlyWheel;
import org.firstinspires.ftc.teamcode.command.MovePosition;
import org.firstinspires.ftc.teamcode.command.TeleopDrive;
import org.firstinspires.ftc.teamcode.drivePose.DrivePose;
import org.firstinspires.ftc.teamcode.subsystem.MyCamera;
import org.firstinspires.ftc.teamcode.subsystem.TestMotor;

@TeleOp
public class MyTeleOp extends CommandOpMode {
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
//    private final Telemetry dashboardTelemetry = dashboard.getTelemetry();
    private final Telemetry dashboardTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    @Override
    public void initialize() {
        DrivePose drive = new DrivePose(hardwareMap, dashboardTelemetry);
        drive.setDefaultCommand(new TeleopDrive(drive, gamepad1));

        TestMotor shooter = new TestMotor(hardwareMap, dashboardTelemetry);
        MyCamera myCamera = new MyCamera(hardwareMap, dashboardTelemetry);
        schedule(new CameraStream(myCamera, gamepad1));

        Button a = new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.A);
        a.whileHeld(new FlyWheel(shooter, 500.0));
        Button b = new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.B);
        b.whileHeld(new FlyWheel(shooter, 1000.0));

        Button x = new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.X);
        x.whenPressed(new MovePosition(shooter,100));
        Button y = new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.Y);
        y.whenPressed(new MovePosition(shooter,800));

    }
}






//        schedule(new RunCommand(dashboardTelemetry::update));
//        FtcDashboard.getInstance().startCameraStream(null, 0);