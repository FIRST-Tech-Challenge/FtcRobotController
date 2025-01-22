package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Sensors.Gyro;
import org.firstinspires.ftc.teamcode.Utils.Vector;
import org.firstinspires.ftc.teamcode.robotSubSystems.Arm.Arm;
import org.firstinspires.ftc.teamcode.robotSubSystems.Arm.ArmState;
import org.firstinspires.ftc.teamcode.robotSubSystems.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.robotSubSystems.ElevatorHorizontical.ElevatorHorizontical;
import org.firstinspires.ftc.teamcode.robotSubSystems.ElevatorHorizontical.ElevatorHorizonticalState;
import org.firstinspires.ftc.teamcode.robotSubSystems.ElevatorVertical.ElevatorVertical;
import org.firstinspires.ftc.teamcode.robotSubSystems.ElevatorVertical.ElevatorVerticalState;
import org.firstinspires.ftc.teamcode.robotSubSystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.robotSubSystems.Intake.IntakeState;
import org.firstinspires.ftc.teamcode.robotSubSystems.Wrist.Wrist;
import org.firstinspires.ftc.teamcode.robotSubSystems.Wrist.WristState;

@TeleOp(name = "TeleOp")
public class TeleOp14029 extends OpMode {
    SubSystemsState systemsState = SubSystemsState.TRAVEL;
    boolean current = true;
    boolean last = false;

    private static final ElapsedTime timer = new ElapsedTime();
    IntakeState intakeState = IntakeState.OFF;
    WristState wristState = WristState.TRANSFER;
    ArmState armState = ArmState.INTAKE;
    ElevatorVerticalState elevatorState = ElevatorVerticalState.INTAKE;
    ElevatorHorizonticalState elevatorHorizonticalState = ElevatorHorizonticalState.OVERRIDE;
    @Override
    public void init() {
        Drivetrain.init(hardwareMap);
        //TODO: check gyro
        Gyro.init(hardwareMap);
        Intake.init(hardwareMap);
        Wrist.init(hardwareMap);
        Arm.init(hardwareMap);
        ElevatorHorizontical.init(hardwareMap);
        ElevatorVertical.init(hardwareMap);
    }
    @Override
    public void loop() {
        //Elevators -------------------------------------------------------------------------
        if (gamepad1.a) {elevatorState = ElevatorVerticalState.INTAKE;}
        if (elevatorState != ElevatorVerticalState.INTAKE) {armState = ArmState.HALF;}else {armState = ArmState.INTAKE;}
        if (gamepad1.b) {elevatorState = ElevatorVerticalState.SPECIMEN;}
        if (gamepad1.x) {elevatorState = ElevatorVerticalState.PUTSPECIMEN;}
        if (gamepad1.y) {elevatorState = ElevatorVerticalState.DEPLETE;}
        if (gamepad1.right_stick_y != 0) {elevatorState = elevatorState.MANUAL;}
        if (gamepad2.start) {ElevatorVertical.resetEncoder(); ElevatorHorizontical.resetEncoder();}
        //.;.;.;.;.;.
        if (gamepad1.dpad_right) {elevatorHorizonticalState = ElevatorHorizonticalState.OPEN;}
        if (gamepad1.dpad_left) {elevatorHorizonticalState = ElevatorHorizonticalState.CLOSE;}
        if (gamepad1.right_stick_x != 0) {elevatorHorizonticalState = ElevatorHorizonticalState.OVERRIDE;}
        //TODO: add reset to horizontal elevs
        //general----------------------------------------------------------------------------
        if (gamepad1.right_bumper) {wristState = WristState.INTAKE; intakeState = IntakeState.IN;}
        if (gamepad1.left_bumper) {wristState = WristState.TRANSFER; intakeState = IntakeState.OFF;}

        if (gamepad1.dpad_down) {intakeState = IntakeState.OUT;}
        if (gamepad1.left_stick_button) {intakeState = IntakeState.OFF;}
        if (gamepad1.dpad_up) {armState = ArmState.DEPLETE;}

        if (gamepad2.x) {intakeState = IntakeState.OUT;}
        if (gamepad2.a) {intakeState = IntakeState.OFF;}


        //TODO: add encoder limiters for horizontal
        ElevatorHorizontical.opreate(elevatorHorizonticalState,gamepad1.right_stick_x,telemetry);
        ElevatorVertical.operate( elevatorState, -gamepad1.right_stick_y,telemetry);
        Intake.operate(intakeState);
        Arm.operate(armState);
        Wrist.operate(wristState);
        //TODO: switch to fieldcentric and add slowedVector
        //Drivetrain.drive(new Vector(-gamepad1.left_stick_x, -gamepad1.left_stick_y), -gamepad1.right_trigger + gamepad1.left_trigger);
        Drivetrain.operate(new Vector(gamepad1.left_stick_x, -gamepad1.left_stick_y), -gamepad1.right_trigger + gamepad1.left_trigger);
        telemetry.addData("right stick y",gamepad1.right_stick_y);
        telemetry.addData("Gyro", Gyro.getAngle());
        telemetry.addData("intakeState", intakeState);
    }
}

