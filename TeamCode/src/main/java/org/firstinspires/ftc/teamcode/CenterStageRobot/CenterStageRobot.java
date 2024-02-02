package org.firstinspires.ftc.teamcode.CenterStageRobot;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CenterStageRobot.commands.ElevatorCommand;
import org.firstinspires.ftc.teamcode.CenterStageRobot.commands.ElevatorManualCommand;
import org.firstinspires.ftc.teamcode.CenterStageRobot.commands.IntakeManualCommand;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.DroneSubsystem;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.IntakeArmSubsystem;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.OuttakeSusystem;
import org.inventors.ftc.robotbase.drive.DriveConstants;
import org.inventors.ftc.robotbase.hardware.GamepadExEx;
import org.inventors.ftc.robotbase.RobotEx;

public class CenterStageRobot extends RobotEx {
    //----------------------------------- Initialize Subsystems -----------------------------------//
    private IntakeArmSubsystem intakeArmSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private OuttakeSusystem outtakeSusystem;
    private ElevatorSubsystem elevatorSubsystem;

    private DroneSubsystem droneSubsystem;

    //----------------------------------- Initialize Commands ------------------------------------//

    public SequentialCommandGroup openOuttake() {
        return new SequentialCommandGroup(
                new InstantCommand(outtakeSusystem::go_outtake_first, outtakeSusystem),
                new WaitCommand(80),
                new InstantCommand(outtakeSusystem::go_outtake_second, outtakeSusystem)
        );
    }

    public SequentialCommandGroup closeOuttake() {
        return new SequentialCommandGroup(
                new InstantCommand(outtakeSusystem::go_intake_second, outtakeSusystem),
                new WaitCommand(80),
                new InstantCommand(outtakeSusystem::go_intake_first, outtakeSusystem)
        );
    }

    public SequentialCommandGroup openExtremeOuttake() {
        return new SequentialCommandGroup(
                new InstantCommand(outtakeSusystem::go_extreme_first, outtakeSusystem),
                new WaitCommand(70),
                new InstantCommand(outtakeSusystem::go_extreme_second, outtakeSusystem)
        );
    }

    public CenterStageRobot(HardwareMap hm, DriveConstants RobotConstants, Telemetry telemetry, GamepadExEx driverOp,
                            GamepadExEx toolOp) {
        super(hm, RobotConstants, telemetry, driverOp, toolOp, OpModeType.TELEOP, false, false);
    }

    public CenterStageRobot(HardwareMap hm, DriveConstants RobotConstants, Telemetry telemetry, GamepadExEx driverOp,
                            GamepadExEx toolOp, OpModeType opModeType, boolean camera, boolean distance_sensor) {
        super(hm, RobotConstants, telemetry, driverOp, toolOp, opModeType, camera, distance_sensor);
    }

    @Override
    public void initMechanismsAutonomous(HardwareMap hardwareMap) {
        super.initMechanismsAutonomous(hardwareMap);
    }

    @Override
    public void initMechanismsTeleOp(HardwareMap hardwareMap) {
        intakeArmSubsystem = new IntakeArmSubsystem(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry);
        outtakeSusystem = new OuttakeSusystem(hardwareMap);
        elevatorSubsystem = new ElevatorSubsystem(hardwareMap, telemetry, () -> toolOp.getLeftY());
        droneSubsystem = new DroneSubsystem(hardwareMap);

//        CommandScheduler.getInstance().registerSubsystem(intakeSubsystem);
//        intakeSubsystem.setDefaultCommand(new IntakeManualCommand(intakeSubsystem, () -> toolOp.getRightY()));

//        toolOp.getGamepadButton(GamepadKeys.Button.B)
//                .toggleWhenPressed(
//                        new InstantCommand(outtakeSusystem::go_outtake, outtakeSusystem),
//                        new InstantCommand(outtakeSusystem::go_intake, outtakeSusystem)
//                );



        toolOp.getGamepadButton(GamepadKeys.Button.B)
                .toggleWhenPressed(openOuttake(), closeOuttake());

        toolOp.getGamepadButton(GamepadKeys.Button.Y)
                        .whenPressed(openExtremeOuttake());

        toolOp.getGamepadButton(GamepadKeys.Button.A)
                .toggleWhenPressed(
                        new InstantCommand(intakeArmSubsystem::lowerArm, intakeArmSubsystem),
                        new InstantCommand(intakeArmSubsystem::raiseArm, intakeArmSubsystem)
                );

        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.Level.LOW));
        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.Level.MID));
        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.Level.HIGH));
        toolOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.Level.HANGING));

        CommandScheduler.getInstance().registerSubsystem(elevatorSubsystem);
        elevatorSubsystem.setDefaultCommand(new ElevatorManualCommand(elevatorSubsystem, toolOp::getLeftY));

        //Intake
        toolOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .toggleWhenPressed(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new InstantCommand(intakeArmSubsystem::lowerArm, intakeArmSubsystem),
                                        new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.Level.LOADING),
                                        closeOuttake()
                                ),
                                new ParallelCommandGroup(
                                        new InstantCommand(intakeSubsystem::run, intakeSubsystem),
                                        new InstantCommand(outtakeSusystem::wheel_grab)
                                )
                        ),
                        new SequentialCommandGroup(
                                new InstantCommand(outtakeSusystem::wheel_stop),
                                new InstantCommand(intakeArmSubsystem::raiseArm),
                                new InstantCommand(intakeSubsystem::reverse),
                                new WaitCommand(1000),
                                new InstantCommand(intakeSubsystem::stop, intakeSubsystem)
                        )
                );



        // Outtake
//        new Trigger(() -> toolOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.8)
//                .whenActive(new SequentialCommandGroup(
//                        new InstantCommand(outtakeSusystem::wheel_release),
//                        new WaitCommand(1400),
//                        new InstantCommand(outtakeSusystem::wheel_stop)
//                ));
//
//        new Trigger(() -> toolOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.8)
//                .whenActive(new SequentialCommandGroup(
//                        new InstantCommand(outtakeSusystem::wheel_release),
//                        new WaitCommand(700),
//                        new InstantCommand(outtakeSusystem::wheel_stop)
//                ));
        new Trigger(() -> toolOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) >= 0.8)
                .whenActive(new InstantCommand(outtakeSusystem::wheel_release));
        new Trigger(() -> toolOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.8)
                .whenActive(new InstantCommand(outtakeSusystem::wheel_stop));

        new Trigger(() -> toolOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.8)
                .whenActive(new InstantCommand(droneSubsystem::release, droneSubsystem));
    }
}