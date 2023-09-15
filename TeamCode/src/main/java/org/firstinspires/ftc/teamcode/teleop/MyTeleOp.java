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
import org.firstinspires.ftc.teamcode.command.AlignAprilTag;
import org.firstinspires.ftc.teamcode.command.FlyWheel;
import org.firstinspires.ftc.teamcode.command.MovePosition;
import org.firstinspires.ftc.teamcode.command.TeleopDrive;
import org.firstinspires.ftc.teamcode.drive.DrivePose;
import org.firstinspires.ftc.teamcode.subsystem.MyCamera;
import org.firstinspires.ftc.teamcode.subsystem.TestMotor;

@TeleOp
public class MyTeleOp extends CommandOpMode {
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private final Telemetry dashboardTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    @Override
    public void initialize() {
        DrivePose drive = new DrivePose(hardwareMap, dashboardTelemetry);
        drive.setDefaultCommand(new TeleopDrive(drive, gamepad1));

        TestMotor shooter = new TestMotor(hardwareMap, dashboardTelemetry);
        MyCamera myCamera = new MyCamera(hardwareMap, dashboardTelemetry);
//        schedule(new CameraStream(myCamera, gamepad1));

        Button a = new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.A);
        a.whileHeld(new FlyWheel(shooter, 500.0));
        Button b = new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.B);
        b.whileHeld(new FlyWheel(shooter, 1000.0));

        Button x = new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.X);
        x.whenPressed(new MovePosition(shooter,100));
        Button y = new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.Y);
        y.whenPressed(new MovePosition(shooter,800));

        Button lb = new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.LEFT_BUMPER);
        lb.whenPressed(new AlignAprilTag(dashboardTelemetry, gamepad1, drive, myCamera, 10.0, 13.216, -0.3125, -0.34));
        Button rb = new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.RIGHT_BUMPER);
        rb.whenPressed(new AlignAprilTag(dashboardTelemetry, gamepad1, drive, myCamera, 10.0, 20.0, 0.0, 0.0));

    }
}






//        schedule(new RunCommand(dashboardTelemetry::update));
//        FtcDashboard.getInstance().startCameraStream(null, 0);