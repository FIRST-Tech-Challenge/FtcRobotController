package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;
@TeleOp(name="ArmTester")
public class ArmTester extends LinearOpMode {
    private Arm arm;
    private GamepadEvents controller;
    private double armPos;
    boolean reverseArm;
    private double[] armPositions = {0, 0.3, 0.5};
//    private double[] armPositions = {.25, -.25, 0};
    private int armPositionIndex = 0;
    private boolean isReversing = false;
    private int armCount = 0;
    private double midArmPos;
    private int testPos = 0;


    @Override
    public void runOpMode() {
        arm = new Arm(hardwareMap, "armRight", "armLeft");
        controller = new GamepadEvents(gamepad1);
        telemetry.addData("Status", "Initialized");

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addLine("OpMode Active");

//            arm.setPosition(Math.pow(controller.left_trigger.getTriggerValue() - controller.right_trigger.getTriggerValue(), 3));
//            arm.setPosition(controller.left_trigger.getTriggerValue() - controller.right_trigger.getTriggerValue());

            midArmPos = 0.5;
            double offset = 0.5;
            double[] testPositions = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};

//            if(controller.x.onPress()) {
//                armPos = testPositions[testPos];
//                arm.setPosition(armPos - offset);
//                testPos++;
//                if(testPos >= testPositions.length)
//                {
//                    testPos = 0;
//                }
//
//            }else if(controller.y.onPress())
//            {
//                armPos =testPositions[testPos];
//                arm.setPosition(armPos);
//                testPos--;
//                if(testPos <= 0)
//                {
//                    testPos = 0;
//                }
//            }
            arm.setPosition(controller.left_trigger.getTriggerValue() - controller.right_trigger.getTriggerValue());
            telemetry.addData("Expected Arm Pos", testPositions[testPos]);

                telemetry.addData("Left Arm Position", arm.getLeftPosition());
                telemetry.addData("Right Arm Position", arm.getRightPosition());
                telemetry.update();
                controller.update();
            }
        }

    }
