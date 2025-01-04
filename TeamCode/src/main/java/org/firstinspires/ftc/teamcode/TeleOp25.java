package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.commands.ArmDown;
import org.firstinspires.ftc.teamcode.commands.ArmHighGoal;
import org.firstinspires.ftc.teamcode.commands.ArmLow;
import org.firstinspires.ftc.teamcode.commands.ArmLowGoal;
import org.firstinspires.ftc.teamcode.commands.ArmMed;
import org.firstinspires.ftc.teamcode.commands.DriveCmd;
import org.firstinspires.ftc.teamcode.commands.IntakeCmd;
import org.firstinspires.ftc.teamcode.commands.MoveArm;
import org.firstinspires.ftc.teamcode.commands.MoveLinearSlide;
import org.firstinspires.ftc.teamcode.commands.ResetLinearSlide;
import org.firstinspires.ftc.teamcode.commands.MoveWristBadlyDown;
import org.firstinspires.ftc.teamcode.commands.MoveWristBadlyUp;
import org.firstinspires.ftc.teamcode.commands.MoveWristLeft;
import org.firstinspires.ftc.teamcode.commands.MoveWristRight;
import org.firstinspires.ftc.teamcode.commands.TurnCmd;
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
    private MoveArm moveArm;
    private ArmLow armLow;
    private ArmMed armMed;
    private ArmLowGoal armLowGoal;
    private ArmHighGoal armHighGoal;

    private LinearSlideSub linearSlideSub;
    private MoveLinearSlide linearSlideUp;
    private MoveLinearSlide linearSlideDown;
    private MoveLinearSlide linearSlideOff;
    private MoveWristBadlyUp wristUp;
    private MoveWristBadlyDown wristDown;
    private MoveLinearSlide moveLinearSlide;
    private ResetLinearSlide resetLinearSlide;
    private TurnCmd turnCW;
    private TurnCmd turnCCW;


    @Override
    public void initialize() {
        robotImu = new ImuSub(hardwareMap, telemetry);

        driverOp = new GamepadEx(gamepad1);
        toolOp = new GamepadEx(gamepad2);


        // MOTOR SYSTEMS
        // Drive
        drive = new DrivetrainSub(hardwareMap, telemetry);
        driveCmd = new DriveCmd(drive, this::rightTrigger, this::leftTrigger, driverOp::getLeftX, driverOp::getLeftY, driverOp::getRightX, driverOp::getRightY, robotImu::getAngle, this::getFieldCentric);

        turnCW = new TurnCmd(-90, 0.4, drive, robotImu, telemetry);
        turnCCW = new TurnCmd(90, 0.4, drive, robotImu, telemetry);

        driverOp.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(this::toggleFieldCentric));

        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(turnCCW);
        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(turnCCW);

        register(drive);
        drive.setDefaultCommand(driveCmd);

        driverOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);

        // Linear Slide
        linearSlideSub = new LinearSlideSub(hardwareMap, telemetry);
        moveLinearSlide = new MoveLinearSlide(linearSlideSub, toolOp::getRightY, telemetry);
        //resetLinearSlide = new ResetLinearSlide(linearSlideSub);

        register(linearSlideSub);
        linearSlideSub.setDefaultCommand(moveLinearSlide);


        // Arm
        armSub = new ArmSub(hardwareMap, telemetry);
        moveArm = new MoveArm(armSub, toolOp::getLeftY, telemetry);
        armLow = new ArmLow(armSub, telemetry);
        armMed = new ArmMed(armSub, telemetry);
        armLowGoal = new ArmLowGoal(armSub, telemetry);
        armHighGoal = new ArmHighGoal(armSub, telemetry);


        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(armLow);
        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(armMed);
        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(armLowGoal);
        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(armHighGoal);

        register(armSub);
        armSub.setDefaultCommand(moveArm);



        // SERVO SYSTEMS
        // Intake
        intake = new IntakeSub(hardwareMap, telemetry);
        intakeIn = new IntakeCmd(intake, 1);
        intakeOut = new IntakeCmd(intake, -1);
        intakeOff = new IntakeCmd(intake, 0);

        toolOp.getGamepadButton(GamepadKeys.Button.A).whenPressed(intakeIn);
        toolOp.getGamepadButton(GamepadKeys.Button.A).whenReleased(intakeOff);
        toolOp.getGamepadButton(GamepadKeys.Button.X).whenPressed(intakeOut);
        toolOp.getGamepadButton(GamepadKeys.Button.X).whenReleased(intakeOff);

        //toolOp.getGamepadButton(GamepadKeys.Button.B).whenPressed(resetLinearSlide);

        // Wrist
        wrist = new WristSub(hardwareMap, telemetry);
        //wristRight = new MoveWristRight(wrist, telemetry);
        //wristLeft = new MoveWristLeft(wrist, telemetry);

        wristUp = new MoveWristBadlyUp(wrist, telemetry);
        wristDown = new MoveWristBadlyDown(wrist, telemetry);

        toolOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(wristUp);
        toolOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(wristDown);



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

    public double rightTrigger() {
        return driverOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
    }

    public double leftTrigger() {
        return driverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
    }
}
