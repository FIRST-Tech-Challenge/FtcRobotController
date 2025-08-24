package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.IntakeClaw;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExt;

public class Teleop extends CommandOpMode {
    private Arm arm;
    private IntakeClaw intakeClaw;
    private GamepadEx driver;
    private IntakeExt intakeExt;
    @Override
    public void initialize() {
        this.arm = new Arm(hardwareMap);
        this.driver = new GamepadEx(this.gamepad1);
        this.intakeClaw = new IntakeClaw(hardwareMap);
        this.intakeExt = new IntakeExt(hardwareMap);




        this.driver.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new SequentialCommandGroup(
                        arm.goToPos(Arm.ArmState.SCORE),
                        arm.armClawOpen(Arm.ArmState.OPEN)
                ))
            .whenReleased(arm.goToPos(Arm.ArmState.HOME));

        this.driver.getGamepadButton(GamepadKeys.Button.A).whenPressed(arm.goToPos(Arm.ArmState.HOME))
                .whenReleased(arm.goToPos(Arm.ArmState.HOME));


        this.driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new ParallelCommandGroup(
                        intakeExt.extendIntake(),
                        intakeClaw.openClaw(),
                        intakeClaw.pivotToReady()
                ))
        .whenReleased(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                intakeClaw.pivotToCollect(),
                                intakeClaw.closeClaw()
                        ),
                        new WaitCommand(700),
                        intakeExt.retractIntake()
                ));


        this.driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(intakeClaw.rotateto90())
                        .whenReleased(intakeClaw.rotateTo0());

        register(arm,intakeClaw, intakeExt);
    }




}
