package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Sensors.Gyro;
import org.firstinspires.ftc.teamcode.Utils.Vector;
import org.firstinspires.ftc.teamcode.robotSubSystems.Arm.Arm;
import org.firstinspires.ftc.teamcode.robotSubSystems.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.robotSubSystems.ElevatorHorizontical.ElevatorHorizontical;
import org.firstinspires.ftc.teamcode.robotSubSystems.ElevatorHorizontical.ElevatorHorizonticalState;
import org.firstinspires.ftc.teamcode.robotSubSystems.ElevatorVertical.ElevatorVertical;
import org.firstinspires.ftc.teamcode.robotSubSystems.ElevatorVertical.ElevatorVerticalState;
import org.firstinspires.ftc.teamcode.robotSubSystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.robotSubSystems.Intake.IntakeState;
import org.firstinspires.ftc.teamcode.robotSubSystems.Wrist.Wrist;
import org.firstinspires.ftc.teamcode.robotSubSystems.Wrist.WristState;

@TeleOp(name = "Test")
public class Test extends OpMode {

    public  static Servo armServo;
    IntakeState intakeState = IntakeState.OFF;
    WristState wristState = WristState.TRANSFER;
    ElevatorVerticalState elevatorState = ElevatorVerticalState.INTAKE;
ElevatorHorizonticalState elevatorHorizonticalState = ElevatorHorizonticalState.OVERRIDE;
    @Override
    public void init() {
        armServo=hardwareMap.servo.get("armServo");
        Drivetrain.init(hardwareMap);
        //Gyro.init(hardwareMap);
        Intake.init(hardwareMap);
        Wrist.init(hardwareMap);
        Arm.init(hardwareMap);
        ElevatorHorizontical.init(hardwareMap);
        ElevatorVertical.init(hardwareMap);
    }
    @Override
    public void loop() {
        if (gamepad1.back) {Gyro.resetGyro();}
        if (gamepad1.dpad_right) {intakeState = intakeState.IN;}
        if (gamepad1.dpad_down) {intakeState = intakeState.OFF;}
        if (gamepad1.dpad_left) {intakeState = intakeState.OUT;}
        if (gamepad1.a) {wristState = wristState.TRANSFER;}
        if (gamepad1.b) {wristState = wristState.INTAKE;}
        if (gamepad1.y) {wristState = wristState.DEPLETE;}
        telemetry.addData("arm position",armServo.getPosition());

//        if (gamepad1.dpad_down) {elevatorState = ElevatorState.INTAKE;}
//        if (gamepad1.dpad_left) {elevatorState = ElevatorState.SPECIMEN;}
//        if (gamepad1.dpad_up) {elevatorState = ElevatorState.PUTSPECIMEN;}
        if (gamepad1.right_stick_y != 0) {elevatorState = elevatorState.MANUAL;}
        if (gamepad1.start) {ElevatorVertical.resetEncoder();}
//        ElevatorHorizontical.opreate(elevatorHorizonticalState,gamepad1.right_stick_x,telemetry);
        ElevatorVertical.operate( elevatorState, gamepad1.right_stick_y, -gamepad2.right_stick_y);
        Intake.operate(intakeState);
        Wrist.operate(wristState);
        Drivetrain.drive(new Vector(-gamepad1.left_stick_x, -gamepad1.left_stick_y), -gamepad1.right_trigger + gamepad1.left_trigger);
        //Drivetrain.operate(new Vector(gamepad1.left_stick_x, -gamepad1.left_stick_y), gamepad1.right_trigger - gamepad1.left_trigger);
        telemetry.addData("right stick y",gamepad1.right_stick_y);
        telemetry.addData("Gyro", Gyro.getAngle());
        telemetry.addData("intakeState", intakeState);
    }
}
