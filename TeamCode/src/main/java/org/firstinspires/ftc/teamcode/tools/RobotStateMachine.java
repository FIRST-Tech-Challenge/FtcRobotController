package org.firstinspires.ftc.teamcode.tools;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ActionBuilder;
import org.firstinspires.ftc.teamcode.Button;

import java.util.function.BooleanSupplier;

public class RobotStateMachine {
    public RobotStateMachine (HardwareMap hardwareMap, Button handlerButtonA, Button handlerButtonB){
        stateMachine = new StateMachine();

        // States
        readyForIntake = new StateMachine.State("readyForIntake");
        intaking = new StateMachine.State("intaking");

        // Adding states to stateMachine
        stateMachine.addState(readyForIntake);
        stateMachine.addState(intaking);

        // Set initial state
        stateMachine.setInitialState(readyForIntake);

        // Motors and servos
        testingMotor = hardwareMap.dcMotor.get("testingMotor");
        testingServo = hardwareMap.servo.get("testingServo");



        // Timer
        timer = new ElapsedTime();
        // Lambda trigger functions
        BooleanSupplier handlerButtonAPressed = handlerButtonA::Pressed;
        BooleanSupplier handlerButtonBPressed = handlerButtonB::Pressed;

        // adding transitions
        readyForIntake.addTransitionTo(intaking, handlerButtonAPressed,
                new ActionBuilder().addLine("start")
                                   .setMotorPosition(testingMotor, 300, 0.7)
                                   .resetTimer(timer)
                                   .waitUntil(timer, 3000)
                                   .addLine("servo has not run")
                                   /*.servoRunToPosition(testingServo, 1.0)*/
                                   .addLine("servo has run"));
        intaking.addTransitionTo(readyForIntake, handlerButtonBPressed,
                new ActionBuilder()/*.servoRunToPosition(testingServo, 0.0)*/
                                   .addLine("Start Message ")
                                   .resetTimer(timer)
                                   .addLine("3rd action ")
                                   .waitUntil(timer, 7000)
                                   .addLine("i skipped ")
                                   .setMotorPosition(testingMotor, 0, 0.7));
    }


    // States
    StateMachine stateMachine;
    StateMachine.State readyForIntake;
    StateMachine.State intaking;
    // Motors and servos
    DcMotor testingMotor;
    Servo testingServo;

    // org.firstinspires.ftc.teamcode.Action Builder
    ActionBuilder actionBuilder;
    ElapsedTime timer;

    public void update(){
        stateMachine.updateState();
        TelemetryManager.getTelemetry().addData("Timer", timer.milliseconds());

    }

}
