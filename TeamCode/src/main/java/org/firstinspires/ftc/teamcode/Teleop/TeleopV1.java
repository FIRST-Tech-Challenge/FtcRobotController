package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.Controller;
import org.firstinspires.ftc.teamcode.Mechanism.LinearSlides;

import java.util.Objects;

@TeleOp(name = "Tele-op")
public class TeleopV1 extends LinearOpMode {

    // declare class variables here
    private Controller controller;
    private FieldCentricDrive fieldCentricDrive;
    private LinearSlides linearSLides;
    // Check if B is pressed
    private boolean b_Press = false;
    public int triggerPressCount = 0;
    private boolean stackState = false;

    public enum TIP {
        TIPPING, NOT_TIPPING, ON_STACKS
    }

    public void runOpMode() {
        telemetry.clear();


        try {
            // setup
            controller = new Controller(gamepad1, gamepad2);
            fieldCentricDrive = new FieldCentricDrive(telemetry, hardwareMap);
            linearSLides = new LinearSlides(telemetry, hardwareMap);


        } catch (Exception exception) {
            telemetry.addLine("Outside of the while loop:");
            telemetry.addLine(exception.getMessage());
            telemetry.update();
            if (Objects.equals(exception.getMessage(), "The IMU was not initialized")) {
                telemetry.addData("-", "Detected");
                telemetry.update();
            }

        }

        telemetry.update();
        linearSLides.init();
        TIP tip = TIP.NOT_TIPPING;
        waitForStart();
        while (opModeIsActive()) {
            try {
                controller.update();
                //FIELDCENTER_______________________________________________________________________________
                double gamepadX;
                double gamepadY;
                double gamepadRot;
                boolean rotationToggle = false;
                boolean strafeToggle = false;
                if (tip == TIP.NOT_TIPPING || tip == TIP.ON_STACKS) {
                    if (Math.abs(controller.gamepad1X) > 0.01) {
                        gamepadX = controller.gamepad1X;
                    } else if (Math.abs(controller.gamepad2X) > 0.01) {
                        gamepadX = controller.gamepad2X;
                    } else {
                        gamepadX = 0;
                    }
                    if (Math.abs(controller.gamepad1Y) > 0.01) {
                        gamepadY = controller.gamepad1Y;
                    } else if (Math.abs(controller.gamepad2Y) > 0.01) {
                        gamepadY = controller.gamepad2Y;
                    } else {
                        gamepadY = 0;
                    }
                    if (Math.abs(controller.gamepad1Rot) > 0.01) {
                        gamepadRot = -controller.gamepad1Rot;
                    } else if (Math.abs(controller.gamepad2Rot) > 0.01) {
                        gamepadRot = -controller.gamepad2Rot;
                    } else {
                        gamepadRot = 0;
                    }
                    fieldCentricDrive.drive(gamepadX, gamepadY, gamepadRot, rotationToggle, strafeToggle);



                }

                if (controller.gamePad1DpadDown){
                    linearSLides.zeroMode = true;
                }

                //Code for each stack Height
                if (controller.y1) {
                    linearSLides.automation = false;
                    stackState = false;
                    triggerPressCount = 0;
                    linearSLides.setHeight(LinearSlides.Ls.HIGH.level);
                    linearSLides.ledTimer.reset();
                } else if (controller.a1) {
                    linearSLides.automation = false;
                    stackState = false;
                    triggerPressCount = 0;
                    linearSLides.setHeight(LinearSlides.Ls.LOW.level);
                    linearSLides.ledTimer.reset();

                } else if (controller.x1) {
                    linearSLides.automation = false;
                    stackState = false;
                    triggerPressCount = 0;
                    linearSLides.setHeight(LinearSlides.Ls.MEDIUM.level);
                    linearSLides.ledTimer.reset();
                } else if (controller.gamePad1RTrigger) {
                    linearSLides.automation = false;
                    stackState = false;
                    triggerPressCount = 0;
                    linearSLides.setHeight(32);
                }

                //This will check if b is pressed if yes then it will check the position of the slides and decide where it should go
                if (controller.b1 & !b_Press) {
                    linearSLides.ledTimer.reset();
                    linearSLides.automation = false;
                    b_Press = true;
                    stackState = false;
                    triggerPressCount = 0;
                    if (linearSLides.target == LinearSlides.Ls.NORM.level) {
                        linearSLides.setHeight(LinearSlides.Ls.IN_CONE.level);
                    } else {
                        linearSLides.setHeight(LinearSlides.Ls.NORM.level);
                    }
                } else {
                    b_Press = false;
                }

                if (controller.gamePad1LTrigger) {
                    if(!stackState && (linearSLides.linearSlides.getTargetPosition() != LinearSlides.Stack.ABOVE_CONE_5.level)){
                        linearSLides.ledTimer.reset();
                        linearSLides.automation = false;
                        linearSLides.setHeight(LinearSlides.Stack.ABOVE_CONE_5.level);
                        stackState = true;
                    } else {
                        linearSLides.automation = true;
                        linearSLides.setGripperPosition(1.0);
                        linearSLides.coneSense();
                        stackState = false;
                    }
                    tip = TIP.ON_STACKS;
                }
                linearSLides.coneSense();

                //GRIPPER__________________________________________________________________________________

                if (controller.gamePad1LBumper && !(controller.gamePad1RBumper)) {
                    triggerPressCount = 0;
                    linearSLides.setGripperPosition(.75);
                }

                if (controller.gamePad1RBumper && !(controller.gamePad1LBumper)) {
                    triggerPressCount = 0;
                    linearSLides.setGripperPosition(1.0);
                }

                //telemetry.addData("-", "tip is activated");

                //float pitch = fieldCentricDrive.getPitch();
                //telemetry.addData("Pitch:", pitch);
                /*if (tip != TIP.ON_STACKS) {
                    if (pitch <= 75) {
                        tip = TIP.TIPPING;
                        fieldCentricDrive.checkifrobotnottipping();
                    } else if (pitch >= 100) {
                        tip = TIP.TIPPING;
                        fieldCentricDrive.checkifrobotnottipping();
                    } else {
                        tip = TIP.NOT_TIPPING;
                        fieldCentricDrive.rightBackMotor.setDirection(DcMotor.Direction.FORWARD);
                        fieldCentricDrive.rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
                        fieldCentricDrive.leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
                        fieldCentricDrive.leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
                    }
                }*/
                fieldCentricDrive.addTelemetry();
                linearSLides.loop();
                linearSLides.zeroSlides();

            } catch (Exception exception) {
                telemetry.addLine("Inside of the while loop:");
                telemetry.clear();
                telemetry.addLine(exception.getMessage());
            }
        }
    }
}