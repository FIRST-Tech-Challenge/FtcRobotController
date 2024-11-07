package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.WormLowerCommand;
import org.firstinspires.ftc.teamcode.commands.WormRaiseCommand;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Worm;

@TeleOp(name="Test: Elevator", group="Test")
public class Test_Elevator extends CommandOpMode {

    private Elevator elevator;
    private Worm worm;

    @Override
    public void initialize() {
        //TODO: figure out why telemetry isn't showing on the robot
        //TODO: configure the
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("intialized", "true");
        elevator = new Elevator(hardwareMap, telemetry);
        worm = new Worm(hardwareMap, telemetry);
        GamepadEx driver = new GamepadEx(gamepad1);
        driver.getGamepadButton(GamepadKeys.Button.A).whileHeld(new InstantCommand(elevator::extend, elevator));
        driver.getGamepadButton(GamepadKeys.Button.B).whileHeld(new InstantCommand(elevator::retract, elevator));
        driver.getGamepadButton(GamepadKeys.Button.X).whenPressed(new WormLowerCommand(worm,1));
        driver.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new WormRaiseCommand(worm, 1));
    }
//
//    @Override
//    public void runOpMode() {
//        //TODO: turn power on elevator to 100% for climbing - test if it can do it
//        //TODO: add driving mechanism to controller for testing
//        //TODO: duplicate elevator code for worm gear
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();
//
//



//        new SequentialCommandGroup(
//                new RunCommand(elevator::extend, elevator),
//                new WaitUntilCommand(() -> elevator.getDistance() >= 50)
//        ).execute();
//
//        waitForStart();
//
//        boolean lastB = false;
//
//        while (opModeIsActive()) {
//            elevator.periodic();
//            if (gamepad1.a) {
//                elevator.setPower((double) gamepad1.left_stick_y);
//                telemetry.addData("Left Stick Y", gamepad1.left_stick_y);
//            }
//            if (gamepad1.b && !lastB) {
//                elevator.retract(1.0);
//            }
//            //if (gamepad1.x) {
//                //elevator.tilt((double) gamepad1.left_stick_y, false);
//            //    telemetry.addData("Left Stick Y", gamepad1.left_stick_y);
//            //}
//            //lastB = gamepad1.b;
//            telemetry.update();
//        }
//    }
}
