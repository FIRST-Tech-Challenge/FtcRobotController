package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Sensors.Gyro;
import org.firstinspires.ftc.teamcode.Utils.Vector;
import org.firstinspires.ftc.teamcode.robotSubSystems.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.robotSubSystems.Elevator.Elevator;
import org.firstinspires.ftc.teamcode.robotSubSystems.Elevator.ElevatorState;
import org.firstinspires.ftc.teamcode.robotSubSystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.robotSubSystems.Intake.IntakeState;

@TeleOp(name = "Test")
public class Test extends OpMode {

    IntakeState intakeState = IntakeState.OFF;
    ElevatorState elevatorState = ElevatorState.INTAKE;
    @Override
    public void init() {
        Drivetrain.init(hardwareMap);
        Gyro.init(hardwareMap);
        Intake.init(hardwareMap);
        Elevator.init(hardwareMap);
    }
    @Override
    public void loop() {
        if (gamepad1.back) {Gyro.resetGyro();}
        if (gamepad1.a) {intakeState = intakeState.IN;}
        if (gamepad1.b) {intakeState = intakeState.OFF;}
        if (gamepad1.y) {intakeState = intakeState.OUT;}
        if (gamepad1.dpad_down) {elevatorState = ElevatorState.INTAKE;}
        if (gamepad1.dpad_left) {elevatorState = ElevatorState.SPECIMEN;}
        if (gamepad1.right_stick_y != 0) {elevatorState = ElevatorState.MANUAL;}

        Elevator.operatee( elevatorState,-gamepad1.right_stick_y);
        Intake.operate(intakeState);
        Drivetrain.operate(new Vector(gamepad1.left_stick_x, -gamepad1.left_stick_y), gamepad1.right_trigger - gamepad1.left_trigger);
        telemetry.addData("Gyro", Gyro.getAngle());
        telemetry.addData("elevPosL", Elevator.leftMotor.getCurrentPosition());
        telemetry.addData("elevPosR", Elevator.rightMotor.getCurrentPosition());
        telemetry.addData("state",elevatorState);
        telemetry.addData("gamepadval", -gamepad1.right_stick_y);
        telemetry.addData("Ltarget", Elevator.leftMotor.getTargetPosition());
        telemetry.addData("Rtarget", Elevator.rightMotor.getTargetPosition());
    }
}
