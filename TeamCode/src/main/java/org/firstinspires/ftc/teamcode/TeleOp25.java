package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.ArmDown;
import org.firstinspires.ftc.teamcode.commands.DriveCmd;
import org.firstinspires.ftc.teamcode.commands.IntakeCmd;
import org.firstinspires.ftc.teamcode.commands.ArmUp;
import org.firstinspires.ftc.teamcode.commands.MoveLinearSlide;
import org.firstinspires.ftc.teamcode.commands.MoveWristBadlyDown;
import org.firstinspires.ftc.teamcode.commands.MoveWristBadlyUp;
import org.firstinspires.ftc.teamcode.commands.MoveWristLeft;
import org.firstinspires.ftc.teamcode.commands.MoveWristRight;
import org.firstinspires.ftc.teamcode.subsystems.ArmSub;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSub;
import org.firstinspires.ftc.teamcode.subsystems.ImuSub;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSub;
import org.firstinspires.ftc.teamcode.subsystems.LinearSlideSub;
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
    private MoveWristLeft wristLeft;
    private MoveWristRight wristRight;

    private ArmSub armSub;
    private ArmUp armUp;
    private ArmDown armDown;

    private LinearSlideSub linearSlideSub;
    private MoveLinearSlide linearSlideUp;
    private MoveLinearSlide linearSlideDown;
    private MoveLinearSlide linearSlideOff;
    private MoveWristBadlyUp wristUp;
    private MoveWristBadlyDown wristDown;


    @Override
    public void initialize() {
        robotImu = new ImuSub(hardwareMap, telemetry);

        driverOp = new GamepadEx(gamepad1);
        toolOp = new GamepadEx(gamepad2);


        // Drive
        drive = new DrivetrainSub(hardwareMap, telemetry);
        driveCmd = new DriveCmd(drive, driverOp, robotImu::getAngle, this::getFieldCentric);

        driverOp.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(this::toggleFieldCentric));

        register(drive);
        drive.setDefaultCommand(driveCmd);


        // Intake
        intake = new IntakeSub(hardwareMap, telemetry);
        intakeIn = new IntakeCmd(intake, 1);
        intakeOut = new IntakeCmd(intake, -1);
        intakeOff = new IntakeCmd(intake, 0);

        toolOp.getGamepadButton(GamepadKeys.Button.X).whenPressed(intakeOut);
        toolOp.getGamepadButton(GamepadKeys.Button.X).whenReleased(intakeOff);
        toolOp.getGamepadButton(GamepadKeys.Button.Y).whenPressed(intakeIn);
        toolOp.getGamepadButton(GamepadKeys.Button.Y).whenReleased(intakeOff);


        // Wrist
        wrist = new WristSub(hardwareMap, telemetry);
        wristRight = new MoveWristRight(wrist, telemetry);
        wristLeft = new MoveWristLeft(wrist, telemetry);

        wristUp = new MoveWristBadlyUp(wrist, telemetry);
        wristDown = new MoveWristBadlyDown(wrist, telemetry);

        toolOp.getGamepadButton(GamepadKeys.Button.A).whenPressed(wristUp);
        toolOp.getGamepadButton(GamepadKeys.Button.A).whenPressed(wristUp);



        // Arm
        armSub = new ArmSub(hardwareMap, telemetry);
        armUp = new ArmUp(armSub, toolOp, telemetry);
        armDown = new ArmDown(armSub, toolOp, telemetry);

        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(armUp);
        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(armUp);
        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenReleased(armUp);
        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenReleased(armDown);


        // Linear Slide
//        linearSlideSub = new LinearSlideSub(hardwareMap, telemetry);
//        linearSlideUp = new MoveLinearSlide(linearSlideSub, telemetry, 0.5);
//        linearSlideDown = new MoveLinearSlide(linearSlideSub, telemetry, -0.5);
//        linearSlideOff = new MoveLinearSlide(linearSlideSub, telemetry, 0);
//
//        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(linearSlideUp);
//        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenReleased(linearSlideOff);
//        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(linearSlideDown);
//        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenReleased(linearSlideOff);
    }

    @Override
    public void run() {
        super.run();

        telemetry.addData("Field Centric?", fieldCentric);
        telemetry.addData("Wrist Position", wrist.getPosition());
//        telemetry.addData("Linear Slide Position", linearSlideSub.getMotor().getCurrentPosition());



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
