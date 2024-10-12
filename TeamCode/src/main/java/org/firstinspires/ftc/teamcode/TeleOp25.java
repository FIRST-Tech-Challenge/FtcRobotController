package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DriveCmd;
import org.firstinspires.ftc.teamcode.commands.IntakeCmd;
import org.firstinspires.ftc.teamcode.commands.WristCmd;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSub;
import org.firstinspires.ftc.teamcode.subsystems.ImuSub;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSub;
import org.firstinspires.ftc.teamcode.subsystems.WristSub;


@TeleOp(name = "Tele-op 2024-25")
public class TeleOp25 extends CommandOpMode {

    private GamepadEx driverOp;
    private GamepadEx toolOp;
    private DrivetrainSub drive;
    private DriveCmd driveCmd;
    private boolean fieldCentric = false;
    private ImuSub robotImu;
    private IntakeSub intake;
    private IntakeCmd intakeIn;
    private IntakeCmd intakeOut;
    private IntakeCmd intakeOff;
    private WristSub wrist;
    private WristCmd wristClockwise;
    private WristCmd wristCounterClockwise;
    private WristCmd wristOff;

    @Override
    public void initialize() {
        robotImu = new ImuSub(hardwareMap, telemetry);

        driverOp = new GamepadEx(gamepad1);
        toolOp = new GamepadEx(gamepad2);

        drive = new DrivetrainSub(hardwareMap, telemetry);
        driveCmd = new DriveCmd(drive, driverOp, robotImu::getAngle, this::getFieldCentric);

        intake = new IntakeSub(hardwareMap, telemetry);
        intakeIn = new IntakeCmd(intake, 1);
        intakeOut = new IntakeCmd(intake, -1);
        intakeOff = new IntakeCmd(intake, 0);

        wrist = new WristSub(hardwareMap, telemetry);
        wristClockwise = new WristCmd(wrist, 1);
        wristCounterClockwise = new WristCmd(wrist, -1);
        wristOff = new WristCmd(wrist, 0);

        // Y: Toggle field centric
        driverOp.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(this::toggleFieldCentric));

        // Intake
        // A: Output Sample
        // B: Take In Sample
        toolOp.getGamepadButton(GamepadKeys.Button.A).whenPressed(intakeOut);
        toolOp.getGamepadButton(GamepadKeys.Button.A).whenReleased(intakeOff);

        toolOp.getGamepadButton(GamepadKeys.Button.B).whenPressed(intakeIn);
        toolOp.getGamepadButton(GamepadKeys.Button.B).whenReleased(intakeOff);

        // Wrist
        // D-Pad Up: Clockwise
        // D-Pad Down: Counter Clockwise
        toolOp.getTrigger(GamepadKeys.Trigger.DPAD_UP).whenPressed(wristClockwise);
        toolOp.getTrigger(GamepadKeys.Trigger.DPAD_UP).whenReleased(wristOff);

        toolOp.getTrigger(GamepadKeys.Trigger.DPAD_DOWN).whenPressed(wristCounterClockwise);
        toolOp.getTrigger(GamepadKeys.Trigger.DPAD_DOWN).whenReleased(wristOff);

        register(drive);
        //register(linearSlide);
        drive.setDefaultCommand(driveCmd);
        //linearSlide.setDefaultCommand(linearSlideCmd);
    }

    @Override
    public void run() {
        super.run();

        telemetry.addData("Field Centric?", fieldCentric);
        telemetry.update();
    }

    public boolean getFieldCentric() {
        return fieldCentric;
    }

    public void toggleFieldCentric() {
        fieldCentric = !fieldCentric;
        if (fieldCentric) {
            robotImu.resetAngle();
        }
    }
}
