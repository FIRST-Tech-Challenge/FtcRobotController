package org.firstinspires.ftc.teamcode.tools;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hanger;
import org.firstinspires.ftc.teamcode.Lift;

import java.util.function.BooleanSupplier;

public class Robot {
    public Robot(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Boolean isAutonomousMode) {

        // Gamepads
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        // Button assignment
        handlerA = new Button(this.gamepad2, Button.NAME.A);
        handlerB = new Button(this.gamepad2, Button.NAME.B);
        handlerX = new Button(this.gamepad2, Button.NAME.X);
        handlerY = new Button(this.gamepad2, Button.NAME.Y);
        handlerLeftBumper = new Button(this.gamepad2, Button.NAME.LEFT_BUMPER);
        handlerRightBumper = new Button(this.gamepad2, Button.NAME.RIGHT_BUMPER);
        handlerLeftTrigger = new Button(this.gamepad2, Button.NAME.LEFT_TRIGGER);
        handlerRightTrigger = new Button(this.gamepad2, Button.NAME.RIGHT_TRIGGER);
        handlerDPad_Down = new Button(this.gamepad2, Button.NAME.DPAD_DOWN);
        handlerDPad_Up = new Button(this.gamepad2, Button.NAME.DPAD_UP);
        handlerDPad_Left = new Button(this.gamepad2, Button.NAME.DPAD_LEFT);
        handlerDPad_Right = new Button(this.gamepad2, Button.NAME.DPAD_RIGHT);


        lift = new Lift(hardwareMap, gamepad2);

        // Motors
        intakeMotor = new OverrideMotor(hardwareMap.dcMotor.get("intakeMotor"));
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        skyHook = new Hanger(hardwareMap, handlerDPad_Down, handlerDPad_Up);//hardwareMap.dcMotor.get("skyHookMotor");


        planeLauncher = hardwareMap.dcMotor.get("planeLauncher");
        planeLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Servos
        clawPitch = hardwareMap.servo.get("clawPitch");
        clawYaw = hardwareMap.servo.get("clawYaw");
        clawGrip = hardwareMap.servo.get("clawGrip");

       // clawGrip.scaleRange(0.02, 0.22);  //old values
        clawGrip.scaleRange(0.05, 0.35);
        clawPitch.scaleRange(0.755, 0.973);
        clawYaw.scaleRange(0.125, 0.675);

        // Touch Sensors
        liftTouchDown = hardwareMap.touchSensor.get("liftTouchDown");

        clawOpen = 1;
        clawClose = 0.30;
        clawCloseOnePixel = 0;
        clawPitchIntake = 1;
        clawPitchOutTake = 0;

        clawYawIntake = 0.5;
        clawYawLeft = 1;
        clawYawRight = 0;


        stateMachine = new StateMachine();

        // States
        intakingPixels = new StateMachine.State("pixelTransition");
        holdingPixels = new StateMachine.State("holdingPixels");
        idle = new StateMachine.State("idle");
        outTakingPixels = new StateMachine.State("outTakingPixels");
        exitingOutTake = new StateMachine.State("exitingOutTake");

        // Adding states to stateMachine
        stateMachine.addState(intakingPixels);
        stateMachine.addState(holdingPixels);
        stateMachine.addState(idle);
        stateMachine.addState(outTakingPixels);
        stateMachine.addState(exitingOutTake);


        // Set initial state
        stateMachine.setInitialState(idle);

        // Timer
        timer = new ElapsedTime();

        if (isAutonomousMode) {
            createAutoStateTransitions();
        }
        else {
            createTeleopStateTransitions();
        }
    }

    public Boolean closeClaw = false;
    public Boolean outtakePixels = false;

    private void createAutoStateTransitions() {
        // button triggers
        BooleanSupplier closeClawSupplier = () -> closeClaw;
        BooleanSupplier openClawSupplier = () -> (!closeClaw);
        BooleanSupplier outtakePixelsSupplier = () -> outtakePixels;

        // adding transitions
        idle.addTransitionTo(holdingPixels, closeClawSupplier,
                new ActionBuilder()
                        .servoRunToPosition(clawGrip, clawCloseOnePixel)
                        .startMotor(lift.liftMotor, 1)
                        .waitForMotorAbovePosition(lift.liftMotor, lift.liftEncoderHolding)
                        .stopMotor(lift.liftMotor));

        // rejecting pixels
//        holdingPixels.addTransitionTo(idle, openClawSupplier,
//                new ActionBuilder()
//                        .servoRunToPosition(clawGrip, clawOpen));

        holdingPixels.addTransitionTo(outTakingPixels, outtakePixelsSupplier,
                new ActionBuilder()
                        .startMotor(lift.liftMotor, 1)
                        .waitForMotorAbovePosition(lift.liftMotor, lift.liftEncoderMin+200)
                        .stopMotor(lift.liftMotor)
                        .servoRunToPosition(clawPitch, clawPitchOutTake));


        outTakingPixels.addTransitionTo(idle, openClawSupplier,
                new ActionBuilder()
                        .servoRunToPosition(clawGrip, clawOpen)
                        .resetTimer(timer)
                        .waitUntil(timer, 300)
                );
    }





    private void createTeleopStateTransitions() {
        // button triggers
        BooleanSupplier handlerButtonAPressed = handlerA::Pressed;
        BooleanSupplier handlerButtonBPressed = handlerB::Pressed;
        BooleanSupplier handlerButtonXPressed = handlerX::Pressed;
        BooleanSupplier handlerButtonYPressed = handlerY::Pressed;
        BooleanSupplier handlerButtonLeftBumperPressed = handlerLeftBumper::Pressed;
        BooleanSupplier handlerButtonRightBumperPressed = handlerRightBumper::Pressed;
        BooleanSupplier handlerButtonLeftTriggerPressed = handlerLeftTrigger::Pressed;
        BooleanSupplier handlerButtonRightTriggerPressed = handlerRightTrigger::Pressed;
        BooleanSupplier handlerDPad_DownPressed = handlerDPad_Down::Pressed;
        BooleanSupplier handlerDPad_UpPressed = handlerDPad_Up::Pressed;
        BooleanSupplier handlerDPad_LeftPressed = handlerDPad_Left::Pressed;
        BooleanSupplier handlerDPad_RightPressed = handlerDPad_Right::Pressed;

        BooleanSupplier alwaysTrue = ()-> true;


        // adding transitions

        // intaking pixels
        idle.addTransitionTo(intakingPixels, handlerButtonAPressed,
                new ActionBuilder()
                        .startMotor(intakeMotor, 0.15));

        intakingPixels.addTransitionTo(holdingPixels, handlerButtonAPressed,
                new ActionBuilder()
                        .servoRunToPosition(clawGrip, clawClose)
                        .resetTimer(timer)
                        .waitUntil(timer, 150)
                        .startMotor(lift.liftMotor, 1)
                        .waitForMotorAbovePosition(lift.liftMotor, lift.liftEncoderHoldingTeleop)
                        .stopMotor(lift.liftMotor)
                        .stopMotor(intakeMotor));

        // rejecting pixels
        holdingPixels.addTransitionTo(idle, handlerButtonLeftTriggerPressed,
                new ActionBuilder()
                        .servoRunToPosition(clawGrip, clawOpen));

        idle.addTransitionTo(holdingPixels, handlerButtonLeftTriggerPressed,
                new ActionBuilder()
                        .servoRunToPosition(clawGrip, clawClose)
                        .resetTimer(timer)
                        .waitUntil(timer, 200)
                        .startMotor(lift.liftMotor, 1)
                        .waitForMotorAbovePosition(lift.liftMotor, lift.liftEncoderHoldingTeleop)
                        .stopMotor(lift.liftMotor));

        holdingPixels.addTransitionTo(outTakingPixels, handlerButtonBPressed,
                new ActionBuilder()
                        .startMotor(lift.liftMotor, 1)
                        .waitForMotorAbovePosition(lift.liftMotor, lift.liftEncoderMin)
                        .stopMotor(lift.liftMotor)
                        .servoRunToPosition(clawPitch, clawPitchOutTake));

        // in exitingOutTake state, lift cannot be controlled manually
        outTakingPixels.addTransitionTo(exitingOutTake, handlerButtonBPressed,
                new ActionBuilder());

        exitingOutTake.addTransitionTo(idle, alwaysTrue,
                new ActionBuilder()
                        // Guarantees lift was not manually put below claw movement limit
                        .startMotor(lift.liftMotor, 1)
                        .waitForMotorAbovePosition(lift.liftMotor, lift.liftEncoderMin)
                        .servoRunToPosition(clawYaw, clawYawIntake)
                        .servoRunToPosition(clawPitch, clawPitchIntake)
                        .startMotor(lift.liftMotor, -1)
                        .waitForTouchSensorPressed(liftTouchDown)
                        .stopMotor(lift.liftMotor)
                        .resetMotorEncoder(lift.liftMotor));
    }


    // States
    StateMachine stateMachine;
    public static Lift lift;
    public StateMachine.State intakingPixels;
    public StateMachine.State holdingPixels;
    public StateMachine.State idle;
    public StateMachine.State outTakingPixels;
    public StateMachine.State exitingOutTake;


    // Motors

    public static OverrideMotor intakeMotor;
    public static Hanger skyHook;
    public static DcMotor planeLauncher;
    // Servos
    public static Servo clawPitch, clawYaw, clawGrip;
    // TouchSensors
    public static TouchSensor liftTouchDown;

    public static double clawPitchIntake, clawPitchOutTake;
    public static double clawOpen, clawClose, clawCloseOnePixel, clawYawIntake, clawYawLeft, clawYawRight;
    int liftOutTake;

    public static Button handlerA, handlerB, handlerX, handlerY, handlerLeftBumper,
            handlerRightBumper, handlerLeftTrigger, handlerRightTrigger, handlerDPad_Down, handlerDPad_Up, handlerDPad_Left, handlerDPad_Right;

    Gamepad gamepad1, gamepad2;
    ActionBuilder actionBuilder;
    ElapsedTime timer;

    private void updateButtons(){
        handlerA.updateButton(gamepad2);
        handlerB.updateButton(gamepad2);
        handlerX.updateButton(gamepad2);
        handlerY.updateButton(gamepad2);
        handlerLeftBumper.updateButton(gamepad2);
        handlerRightBumper.updateButton(gamepad2);
        handlerLeftTrigger.updateButton(gamepad2);
        handlerRightTrigger.updateButton(gamepad2);
        handlerDPad_Down.updateButton(gamepad2);
        handlerDPad_Up.updateButton(gamepad2);
        handlerDPad_Left.updateButton(gamepad2);
        handlerDPad_Right.updateButton(gamepad2);
    }

    public void updateSync() {
        stateMachine.updateStateSync(); //Do one call to process potential trigger
    }

    public void update(){
        updateButtons();

        // Manages Reject mode on Roomba as an override of its current power and state
        if(handlerRightTrigger.Pressed()) {
            intakeMotor.setOverridePower(-0.4);
        } else if (handlerRightTrigger.Released()) {
            intakeMotor.cancelOverridePower();
        }
        if(handlerX.On()) {
            planeLauncher.setPower(1);
        }
        else{
            planeLauncher.setPower(0);
        }
        skyHook.update(handlerDPad_Down);


        stateMachine.updateState();

        if(stateMachine.getCurrentState() == outTakingPixels){
            lift.update();
        }
        //lift.update();
        TelemetryManager.getTelemetry().addLine(""+lift.liftMotor.getCurrentPosition());

    }

    public StateMachine.State currentState(){
        return stateMachine.getCurrentState();
    }

}
