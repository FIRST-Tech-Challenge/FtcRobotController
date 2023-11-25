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

        // States
        intakingPixels = new StateMachine.State("pixelTransition");
        rejectingPixels = new StateMachine.State("rejectingPixels");
        holdingPixels = new StateMachine.State("holdingPixels");
        noPixels = new StateMachine.State("noPixels");
        outTakingPixels = new StateMachine.State("outTakingPixels");

        // Adding states to stateMachine
        stateMachine.addState(intakingPixels);
        stateMachine.addState(rejectingPixels);
        stateMachine.addState(holdingPixels);
        stateMachine.addState(noPixels);
        stateMachine.addState(outTakingPixels);


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
        intakeMotor = new OverrideMotor(hardwareMap.dcMotor.get("intakeMotor"));
        liftMotor = hardwareMap.dcMotor.get("liftMotor");
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

        clawPitchIntake = 0;
        clawPitchOutTake = 0;

        clawYawIntake = -0.5; // temporary placeholder value, even though servo pos is 0 to 1 still need to reprogram it to turn further




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



        // testing
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


        // adding transitions

        // intaking pixels
        /*noPixels.addTransitionTo(intakingPixels, handlerButtonAPressed,
                new ActionBuilder()
                        .startMotor(intakeMotor, 0.2));

        intakingPixels.addTransitionTo(holdingPixels, handlerButtonAPressed,
                new ActionBuilder()
                        .servoRunToPosition(clawGrip, clawClose)
                        .stopMotor(intakeMotor));

        // rejecting pixels
        holdingPixels.addTransitionTo(rejectingPixels, handlerButtonLeftBumperPressed,
                new ActionBuilder()
                        .servoRunToPosition(clawGrip, clawOpen)
                        .startMotor(intakeMotor, -0.2));

        rejectingPixels.addTransitionTo(noPixels, handlerButtonLeftBumperPressed,
                new ActionBuilder()
                        .stopMotor(intakeMotor));*/

        holdingPixels.addTransitionTo(outTakingPixels, handlerButtonBPressed,
                new ActionBuilder()
                        .setMotorPosition(liftMotor, liftOutTake, 1)
                        .servoRunToPosition(clawPitch, clawPitchOutTake));

        //outTakingPixels.addTransitionTo(noPixels);
    }



    // States
    StateMachine stateMachine;
    /*StateMachine.State readyForIntake;
    StateMachine.State intaking;*/
    public StateMachine.State intakingPixels, rejectingPixels, holdingPixels, noPixels, outTakingPixels;

    // testing
    /*DcMotor testingMotor;
    Servo testingServo;*/

    // Motors

    public static OverrideMotor intakeMotor;
    public static DcMotor liftMotor, skyHookMotor;
    // Servos
    public static Servo clawPitch, clawYaw, clawGrip;
    // TouchSensors
    public static TouchSensor liftTouchDown, skyHookTouchUp;

    static double clawPitchIntake, clawPitchOutTake;
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
        if(handlerRightTrigger.Pressed()) {
            intakeMotor.setOverridePower(-1);
        } else if (handlerRightTrigger.Released()) {
            intakeMotor.cancelOverridePower();
        }
        stateMachine.updateState();
    }

    public StateMachine.State currentState(){
        return stateMachine.getCurrentState();
    }

}
