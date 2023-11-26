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
import org.firstinspires.ftc.teamcode.Lift;
import org.firstinspires.ftc.teamcode.OverrideMotor;

import java.util.function.BooleanSupplier;

public class Robot {
    public Robot(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2){

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

        stateMachine = new StateMachine();

        lift = new Lift(hardwareMap, gamepad2);

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


        // Motors
        intakeMotor = new OverrideMotor(hardwareMap.dcMotor.get("intakeMotor"));
        skyHookMotor = hardwareMap.dcMotor.get("skyHookMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Servos
        clawPitch = hardwareMap.servo.get("clawPitch");
        clawYaw = hardwareMap.servo.get("clawYaw");
        clawGrip = hardwareMap.servo.get("clawGrip");

        clawGrip.scaleRange(0.02, 0.22);
        clawPitch.scaleRange(0.755, 0.973);
        clawYaw.scaleRange(0.125, 0.675);

        // Touch Sensors
        liftTouchDown = hardwareMap.touchSensor.get("liftTouchDown");
        skyHookTouchUp = hardwareMap.touchSensor.get("skyHookTouchUp");

        clawOpen = 1;
        clawClose = 0;

        clawPitchIntake = 1;
        clawPitchOutTake = 0;

        clawYawIntake = 0.5;
        clawYawLeft = 1;
        clawYawRight = 0;




        // Timer
        timer = new ElapsedTime();

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
                        .startMotor(intakeMotor, 0.2));

        intakingPixels.addTransitionTo(holdingPixels, handlerButtonAPressed,
                new ActionBuilder()
                        .servoRunToPosition(clawGrip, clawClose)
                        .stopMotor(intakeMotor));

        // rejecting pixels
        holdingPixels.addTransitionTo(idle, handlerDPad_UpPressed,
                new ActionBuilder()
                        .servoRunToPosition(clawGrip, clawOpen));

        idle.addTransitionTo(holdingPixels, handlerDPad_UpPressed,
                new ActionBuilder()
                        .servoRunToPosition(clawGrip, clawClose));

        holdingPixels.addTransitionTo(outTakingPixels, handlerButtonBPressed,
                new ActionBuilder()
                        .startMotor(lift.liftMotor, 1)
                        .waitForMotorAbovePosition(lift.liftMotor, lift.liftEncoderMin)
                        //.setMotorPosition(lift.liftMotor, lift.liftEncoderMin, 1)
                        //.waitFor(3000)
                        .stopMotor(lift.liftMotor)
                        .servoRunToPosition(clawPitch, clawPitchOutTake));

        outTakingPixels.addTransitionTo(exitingOutTake, handlerButtonBPressed,
                new ActionBuilder());

        exitingOutTake.addTransitionTo(idle, alwaysTrue,
                new ActionBuilder()
                        .servoRunToPosition(clawYaw, clawYawIntake)
                        .servoRunToPosition(clawPitch, clawPitchIntake)
                        .startMotor(lift.liftMotor, -1)
                        .waitForTouchSensorPressed(liftTouchDown)
                        .stopMotor(lift.liftMotor)
                        .resetMotorEncoder(lift.liftMotor));
    }



    // States
    StateMachine stateMachine;
    Lift lift;
    public StateMachine.State intakingPixels;
    public StateMachine.State holdingPixels;
    public StateMachine.State idle;
    public StateMachine.State outTakingPixels;
    public StateMachine.State exitingOutTake;


    // Motors

    public static OverrideMotor intakeMotor;
    public static DcMotor skyHookMotor;
    // Servos
    public static Servo clawPitch, clawYaw, clawGrip;
    // TouchSensors
    public static TouchSensor liftTouchDown, skyHookTouchUp;

    public static double clawPitchIntake, clawPitchOutTake;
    public static double clawOpen, clawClose, clawYawIntake, clawYawLeft, clawYawRight;
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

    public void update(){
        updateButtons();

        // Manages Reject mode on Roomba as an override of its current power and state
        if(handlerRightTrigger.Pressed()) {
            intakeMotor.setOverridePower(-0.4);
        } else if (handlerRightTrigger.Released()) {
            intakeMotor.cancelOverridePower();
        }
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
