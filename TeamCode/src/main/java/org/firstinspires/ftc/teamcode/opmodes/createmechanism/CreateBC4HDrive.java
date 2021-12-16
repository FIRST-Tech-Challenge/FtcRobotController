package org.firstinspires.ftc.teamcode.opmodes.createmechanism;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.drive.bc4h.DefaultDrive;
import org.firstinspires.ftc.teamcode.commands.intake.MoveIntake;
import org.firstinspires.ftc.teamcode.subsystems.drive.bc4h.BC4HDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;

public class CreateBC4HDrive {

    private BC4HDriveSubsystem drive;
    private final HardwareMap hwMap;
    private final String deviceNameFl;
    private final String deviceNameFr;
    private final String deviceNameBl;
    private final String deviceNameBr;
    private final Telemetry telemetry;
    private final GamepadEx op;

    private static final int REV_ENCODER_CLICKS = 560;
    private static final double REV_WHEEL_DIAM = 7.5;

    public CreateBC4HDrive(final HardwareMap hwMap, final String deviceNameFl,final String deviceNameFr, final String deviceNameBl,final String deviceNameBr, final GamepadEx op, Telemetry telemetry){
        this.deviceNameFl = deviceNameFl;
        this.deviceNameFr = deviceNameFr;
        this.deviceNameBl = deviceNameBl;
        this.deviceNameBr = deviceNameBr;
        this.hwMap = hwMap;
        this.op = op;
        this.telemetry = telemetry;

    }

    public CreateBC4HDrive(final HardwareMap hwMap, final String deviceNameFl,final String deviceNameFr, final String deviceNameBl,final String deviceNameBr, final GamepadEx op, Telemetry telemetry, boolean autoCreate){
        this.deviceNameFl = deviceNameFl;
        this.deviceNameFr = deviceNameFr;
        this.deviceNameBl = deviceNameBl;
        this.deviceNameBr = deviceNameBr;
        this.hwMap = hwMap;
        this.op = op;
        this.telemetry = telemetry;

        if (autoCreate) create();

    }

    public void create(){

        drive = new BC4HDriveSubsystem(hwMap, deviceNameFl, deviceNameBl, deviceNameFr, deviceNameBr,
                REV_ENCODER_CLICKS, REV_WHEEL_DIAM, DcMotor.Direction.FORWARD, DcMotor.Direction.FORWARD,
                DcMotor.Direction.REVERSE, DcMotor.Direction.REVERSE, DcMotor.RunMode.RUN_USING_ENCODER );

        DefaultDrive driveRobot = new DefaultDrive(drive, () -> op.getLeftY(), () -> op.getLeftX(), () -> op.getRightX(), () -> op.isDown(GamepadKeys.Button.B));
        drive.setDefaultCommand(driveRobot);
    }
}
