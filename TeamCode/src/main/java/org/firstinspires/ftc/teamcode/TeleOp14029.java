package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Sensors.Gyro;
import org.firstinspires.ftc.teamcode.Utils.Delay;
import org.firstinspires.ftc.teamcode.Utils.GlobalData;
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
    ElapsedTime elapsedTime = new ElapsedTime();
    Delay delay = new Delay(1);
    boolean firstTimePressedleft_bumper;
    float left_bumperTimer = 0;

    boolean firstTimePressedA;
    float aTimer;
    IntakeState intakeState = IntakeState.OFF;
    WristState wristState = WristState.TRANSFER;
    ArmState armState = ArmState.INTAKE;
    ElevatorVerticalState elevatorState = ElevatorVerticalState.INTAKE;
    ElevatorHorizonticalState elevatorHorizonticalState = ElevatorHorizonticalState.ALMOST;
    @Override
    public void init() {
        Drivetrain.init(hardwareMap);
        Gyro.init(hardwareMap);
        Intake.init(hardwareMap);
        Wrist.init(hardwareMap);
        Arm.init(hardwareMap);
        ElevatorHorizontical.init(hardwareMap);
        ElevatorVertical.init(hardwareMap);
        elapsedTime.reset();
    }
    @Override
    public void loop() {
        GlobalData.currentTime = (float) elapsedTime.milliseconds();


        //vertical elevators -------------------------------------------------------------------------
        if (gamepad1.a) {elevatorState = ElevatorVerticalState.INTAKE; elevatorHorizonticalState = ElevatorHorizonticalState.HALF;
            firstTimePressedA = true;
            aTimer = GlobalData.currentTime;
        }
        if ((GlobalData.currentTime - aTimer) > 1000 && firstTimePressedA == true) {
            elevatorHorizonticalState = ElevatorHorizonticalState.ALMOST;
            firstTimePressedA = false;
        }


        if (elevatorState != ElevatorVerticalState.INTAKE && ElevatorVertical.getElevatorPos() > 340) {armState = ArmState.HALF;}else {armState = ArmState.INTAKE;}
        if (gamepad1.b) {elevatorState = ElevatorVerticalState.SPECIMEN;}
        if (gamepad1.x) {elevatorState = ElevatorVerticalState.PUTSPECIMEN;}
        if (gamepad1.y) {elevatorState = ElevatorVerticalState.DEPLETE;}
        if (gamepad1.right_stick_y != 0) {elevatorState = elevatorState.MANUAL;}
        if (gamepad1.start) {ElevatorVertical.resetEncoder(); ElevatorHorizontical.resetEncoder();}
        //horizontal elevators ------------------------------------------------------------------------
        if (gamepad1.right_stick_x != 0) {elevatorHorizonticalState = ElevatorHorizonticalState.OVERRIDE;}

        if (gamepad1.dpad_down) {
            wristState = WristState.INTAKE; intakeState = IntakeState.IN;
            elevatorHorizonticalState = ElevatorHorizonticalState.OPEN;
        }

        if (gamepad1.left_bumper) {
            firstTimePressedleft_bumper =true;
            wristState = WristState.TRANSFER; intakeState = IntakeState.OFF;
            elevatorHorizonticalState = ElevatorHorizonticalState.ALMOST;
            left_bumperTimer = GlobalData.currentTime;
        }
        if ((GlobalData.currentTime - left_bumperTimer) > 1100 && (GlobalData.currentTime-left_bumperTimer) < 4000 && firstTimePressedleft_bumper) {
            intakeState = IntakeState.OUT;
            firstTimePressedleft_bumper = false;
        }
        //general----------------------------------------------------------------------------
        if (gamepad1.right_bumper) {elevatorHorizonticalState = ElevatorHorizonticalState.HALF; wristState = WristState.INTAKE; intakeState = IntakeState.IN;}
        if (gamepad1.left_bumper) {wristState = WristState.TRANSFER; intakeState = IntakeState.OFF;}

        if (gamepad1.right_stick_button) {intakeState = IntakeState.OUT;}
        if (gamepad1.left_stick_button) {intakeState = IntakeState.OFF;}
        if (gamepad1.dpad_up) {armState = ArmState.DEPLETE;}
        if (gamepad1.dpad_left || gamepad1.dpad_right) {wristState = WristState.DEPLETE;}

        if (gamepad2.x) {intakeState = IntakeState.OUT;}
        if (gamepad2.a) {intakeState = IntakeState.OFF;}

        if (gamepad1.back) {Gyro.resetGyro();}

        //TODO: add encoder limiters for horizontal
        ElevatorHorizontical.opreate(elevatorHorizonticalState,gamepad1.right_stick_x,telemetry);
        ElevatorVertical.operate( elevatorState, -gamepad1.right_stick_y,telemetry);
        Intake.operate(intakeState);
        Arm.operate(armState);
        Wrist.operate(wristState);
        Drivetrain.operate(new Vector(gamepad1.left_stick_x, -gamepad1.left_stick_y), -gamepad1.right_trigger + gamepad1.left_trigger);
        telemetry.addData("Gyro", Gyro.getAngle());
        telemetry.addData("horizontalPos", ElevatorHorizontical.getPos());
        telemetry.addData("verticalPos", ElevatorVertical.getElevatorPos());
        telemetry.addData("timer", GlobalData.currentTime);
        telemetry.addData("timer 2", GlobalData.currentTime - left_bumperTimer);
        telemetry.addData("bool", firstTimePressedleft_bumper);
    }
}

