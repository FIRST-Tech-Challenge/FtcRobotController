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
//    private double[] cycledArmPositions = {0.4, 0.3, 0.1, 0.0};
    private double[] armPositions = {0.3, 0.0, 0.5};
    private int armPositionIndex = 0;
    private boolean isReversing = false;

    @Override
    public void runOpMode(){
        arm = new Arm(hardwareMap);
        controller = new GamepadEvents(gamepad1);
        telemetry.addData("Status","Initialized");

        waitForStart();
        while(opModeIsActive()){
            telemetry.addLine("OpMode Active");

            if(controller.right_bumper.onPress())
            {
                armPositionIndex = 0;
                arm.setPosition(armPositions[armPositionIndex]);
                sleep(1000);
                armPositionIndex = 1;
                arm.setPosition(armPositions[armPositionIndex]);
            }else if(controller.left_bumper.onPress())
            {
                armPositionIndex = 2;
                arm.setPosition(armPositions[armPositionIndex]);

            }
//            if(controller.right_bumper.onPress())
//            {
//                if(!isReversing)
//                {
//                    armPos = armPositions[armPositionIndex];
//                    arm.setPosition(armPos);
//                    armPositionIndex++;
//                    if(armPositionIndex >= armPositions.length)
//                    {
//                        armPositionIndex--;
//                        isReversing = true;
//                    }
//                }else {
//                    armPos = armPositions[armPositionIndex];
//                    arm.setPosition(armPos);
//                    armPositionIndex--;
//                    if(armPositionIndex < 0)
//                    {
//                        armPositionIndex++;
//                        isReversing = false;
//                    }
//                }
//
//
//            }
//            if(controller.left_bumper.onPress())
//            {
//
//                armPos = armPositions[armPositionIndex];
//                arm.setPosition(armPos);
//                armPositionIndex--;
//                if(armPositionIndex < 0)
//                {
//                    armPositionIndex = armPositions.length-1;
//                }
//
//            }
            telemetry.addData("Arm Position", armPos);
            telemetry.update();
            controller.update();
        }
    }

}
