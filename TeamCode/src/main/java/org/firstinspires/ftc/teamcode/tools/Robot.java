package org.firstinspires.ftc.teamcode.tools;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ActionBuilder;
import org.firstinspires.ftc.teamcode.Button;

import java.util.function.BooleanSupplier;

public class Robot {
    public Robot(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2){

        // Gamepads
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        // Button assignment
        handlerButtonA = new Button(this.gamepad2, Button.NAME.A);
        handlerButtonB = new Button(this.gamepad2, Button.NAME.B);
        handlerButtonX = new Button(this.gamepad2, Button.NAME.X);
        handlerButtonY = new Button(this.gamepad2, Button.NAME.Y);
        handlerButtonLeftBumper = new Button(this.gamepad2, Button.NAME.LEFT_BUMPER);
        handlerButtonRightBumper = new Button(this.gamepad2, Button.NAME.RIGHT_BUMPER);
        handlerButtonLeftTrigger = new Button(this.gamepad2, Button.NAME.LEFT_TRIGGER);
        handlerButtonRightTrigger = new Button(this.gamepad2, Button.NAME.RIGHT_TRIGGER);

        stateMachine = new StateMachine();

        // States
        intakingPixels = new StateMachine.State("pixelTransition");
        rejectingPixels = new StateMachine.State("rejectingPixels");
        holdingPixels = new StateMachine.State("holdingPixels");
        noPixels = new StateMachine.State("noPixels");

        // Adding states to stateMachine
        stateMachine.addState(intakingPixels);
        stateMachine.addState(rejectingPixels);
        stateMachine.addState(holdingPixels);
        stateMachine.addState(noPixels);

        //testing
        /*readyForIntake = new StateMachine.State("readyForIntake");
        intaking = new StateMachine.State("intaking");
        stateMachine.addState(readyForIntake);
        stateMachine.addState(intaking);*/



        // Set initial state
        stateMachine.setInitialState(noPixels);

        // testing
        /*testingMotor = hardwareMap.dcMotor.get("testingMotor");
        testingServo = hardwareMap.servo.get("testingServo");*/

        // Motors
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        liftMotor = hardwareMap.dcMotor.get("liftMotor");
        skyHookMotor = hardwareMap.dcMotor.get("skyHookMotor");

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Servos
        clawPitch = hardwareMap.servo.get("clawPitch");
        clawYaw = hardwareMap.servo.get("clawYaw");
        clawGrip = hardwareMap.servo.get("clawGrip");

        // Touch Sensors
        liftTouchDown = hardwareMap.touchSensor.get("liftTouchDown");
        skyHookTouchUp = hardwareMap.touchSensor.get("skyHookTouchUp");

        clawOpen = 1;
        clawClose = 0;



        // Timer
        timer = new ElapsedTime();

        // button triggers
        BooleanSupplier handlerButtonAPressed = handlerButtonA::Pressed;
        BooleanSupplier handlerButtonBPressed = handlerButtonB::Pressed;
        BooleanSupplier handlerButtonXPressed = handlerButtonX::Pressed;
        BooleanSupplier handlerButtonYPressed = handlerButtonY::Pressed;
        BooleanSupplier handlerButtonLeftBumperPressed = handlerButtonLeftBumper::Pressed;
        BooleanSupplier handlerButtonRightBumperPressed = handlerButtonRightBumper::Pressed;
        BooleanSupplier handlerButtonLeftTriggerPressed = handlerButtonLeftTrigger::Pressed;
        BooleanSupplier handlerButtonRightTriggerPressed = handlerButtonRightTrigger::Pressed;


        // adding transitions
        /*readyForIntake.addTransitionTo(intaking, handlerButtonAPressed,
                new ActionBuilder().addLine("start")
                                   .setMotorPosition(testingMotor, 300, 0.7)
                                   .resetTimer(timer)
                                   .waitUntil(timer, 3000)
                                   .addLine("servo has not run")
                                   .servoRunToPosition(testingServo, 1.0)
                                   .addLine("servo has run"));
        intaking.addTransitionTo(readyForIntake, handlerButtonBPressed,
                new ActionBuilder().servoRunToPosition(testingServo, 0.0)
                                   .addLine("Start Message ")
                                   .resetTimer(timer)
                                   .addLine("3rd action ")
                                   .waitUntil(timer, 7000)
                                   .addLine("i skipped ")
                                   .setMotorPosition(testingMotor, 0, 0.7));
        */
        noPixels.addTransitionTo(intakingPixels, handlerButtonAPressed,
                new ActionBuilder()
                        .startMotor(intakeMotor, 0.2));

        intakingPixels.addTransitionTo(holdingPixels, handlerButtonAPressed,
                new ActionBuilder()
                        .servoRunToPosition(clawGrip, clawClose)
                        .stopMotor(intakeMotor));

        holdingPixels.addTransitionTo(rejectingPixels, handlerButtonLeftBumperPressed,
                new ActionBuilder()
                        .servoRunToPosition(clawGrip, clawOpen)
                        .startMotor(intakeMotor, -0.2));

        rejectingPixels.addTransitionTo(noPixels, handlerButtonLeftBumperPressed,
                new ActionBuilder()
                        .stopMotor(intakeMotor));
    }



    // States
    StateMachine stateMachine;
    /*StateMachine.State readyForIntake;
    StateMachine.State intaking;*/
    StateMachine.State intakingPixels;
    StateMachine.State rejectingPixels;
    StateMachine.State holdingPixels;
    StateMachine.State noPixels;

    // testing
    /*DcMotor testingMotor;
    Servo testingServo;*/

    // Motors

    DcMotor intakeMotor;
    DcMotor liftMotor;
    DcMotor skyHookMotor;
    // Servos
    Servo clawPitch;
    Servo clawYaw;
    Servo clawGrip;
    // TouchSensors
    TouchSensor liftTouchDown;
    TouchSensor skyHookTouchUp;

    double clawOpen;
    double clawClose;

    public Button handlerButtonA, handlerButtonB, handlerButtonX, handlerButtonY, handlerButtonLeftBumper,
            handlerButtonRightBumper, handlerButtonLeftTrigger, handlerButtonRightTrigger;

    Gamepad gamepad1;
    Gamepad gamepad2;
    ActionBuilder actionBuilder;
    ElapsedTime timer;

    private void updateButtons(){
        handlerButtonA.updateButton(gamepad2);
        handlerButtonB.updateButton(gamepad2);
        handlerButtonX.updateButton(gamepad2);
        handlerButtonY.updateButton(gamepad2);
        handlerButtonLeftBumper.updateButton(gamepad2);
        handlerButtonRightBumper.updateButton(gamepad2);
        handlerButtonLeftTrigger.updateButton(gamepad2);
        handlerButtonRightTrigger.updateButton(gamepad2);
    }

    public void update(){
        updateButtons();
        stateMachine.updateState();
    }

}
