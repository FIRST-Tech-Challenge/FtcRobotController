/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

//import org.firstinspires.ftc.teamcode.Drivercontrol.drive.Feildcentricdrive;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ftcLib_DLC.TriggerAnalogButton;
import org.firstinspires.ftc.teamcode.robot.commands.claw.ClawCloseCommand;
import org.firstinspires.ftc.teamcode.robot.commands.claw.ClawOpenCommand;
import org.firstinspires.ftc.teamcode.robot.commands.drivetrain.DriveFieldCentric;
import org.firstinspires.ftc.teamcode.robot.commands.drivetrain.ResetIMU;
import org.firstinspires.ftc.teamcode.robot.commands.extension.ExtensionGoToPosition;
import org.firstinspires.ftc.teamcode.robot.commands.extension.ExtensionJoystick;
import org.firstinspires.ftc.teamcode.robot.commands.plane_launcher.LaunchPlane;
import org.firstinspires.ftc.teamcode.robot.commands.tilt.TiltGoToPosition;
import org.firstinspires.ftc.teamcode.robot.commands.wrist.WristDeposit;
import org.firstinspires.ftc.teamcode.robot.commands.wrist.WristIntake;
import org.firstinspires.ftc.teamcode.robot.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.PlaneLauncherSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.TiltSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.WristSubsystem;

import java.util.function.DoubleSupplier;

@TeleOp(name="The Best Teleop Known To Mankind", group="Linear OpMode")

public class TheBestTeleopKnownToMankind extends CommandOpMode
{
    DriveSubsystem driveSubsystem;
    @Override
    public void initialize()
    {
        GamepadEx driver = new GamepadEx(gamepad1);
        GamepadEx operator = new GamepadEx(gamepad2);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ClawSubsystem clawSubsystem = new ClawSubsystem(hardwareMap);
        TiltSubsystem tiltSubsystem = new TiltSubsystem(hardwareMap, telemetry);
        WristSubsystem wristSubsystem = new WristSubsystem(hardwareMap);
        PlaneLauncherSubsystem planeLauncherSubsystem = new PlaneLauncherSubsystem(hardwareMap);
      driveSubsystem = new DriveSubsystem(hardwareMap);
      //  ExtensionSubsystem extensionSubsystem = new ExtensionSubsystem(hardwareMap);

        //driver
        TriggerAnalogButton driverTrigger =
                new TriggerAnalogButton(operator,GamepadKeys.Trigger.LEFT_TRIGGER,0.9);

        driveSubsystem.setDefaultCommand(
                new DriveFieldCentric(
                driveSubsystem,
                () -> driver.getLeftY(),
                () -> driver.getLeftX(),
                () -> driver.getRightX(),
                () -> driverTrigger.get()));

        //manual extension by default unless another command using it runs
//        extensionSubsystem.setDefaultCommand(
//                new ExtensionJoystick(extensionSubsystem, () -> operator.getLeftY()));

        //claw
        TriggerAnalogButton clawTrigger =
                new TriggerAnalogButton(operator, GamepadKeys.Trigger.RIGHT_TRIGGER,0.9);
        clawTrigger.whenReleased(
                new ClawCloseCommand(clawSubsystem));
        clawTrigger.whenPressed(
                new ClawOpenCommand(clawSubsystem));
        //reset heading
        driver.getGamepadButton(GamepadKeys.Button.BACK).whenPressed(new ResetIMU(driveSubsystem));

        //deposit
        operator.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new SequentialCommandGroup(
                        new TiltGoToPosition(tiltSubsystem, TiltGoToPosition.TELEOP_DEPOSIT),
                        new WristDeposit(wristSubsystem)));

        //intake
        operator.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new SequentialCommandGroup(
                        new TiltGoToPosition(tiltSubsystem, TiltGoToPosition.TELEOP_INTAKE),
                  //      new ExtensionGoToPosition(extensionSubsystem, ExtensionGoToPosition.STOW_POSITION),
                        new WristIntake(wristSubsystem)));

        //plane launcher
        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .and(operator.getGamepadButton(GamepadKeys.Button.A))
                .whenActive(new LaunchPlane(planeLauncherSubsystem));

        // should be able to get interrupted by ExtensionGoToPosition
        //CommandScheduler.getInstance().schedule(true,extendoManualCommand);

    }

    @Override
    public void run()
    {
        telemetry.addData("heading", driveSubsystem.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        super.run();
        telemetry.update();
    }
}


