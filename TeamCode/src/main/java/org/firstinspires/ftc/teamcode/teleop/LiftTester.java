package org.firstinspires.ftc.teamcode.teleop;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;
@TeleOp(name = "LiftTester")
public class LiftTester extends LinearOpMode {
    private Lift lift;
    private GamepadEvents controller;
    private int expectedManualLiftPos;
    private int specimenScore, bucketScore, climb;
    private double liftPower;
    public void runOpMode(){
        expectedManualLiftPos = 0;
        lift = new Lift(hardwareMap, "lift", "lift");
        controller = new GamepadEvents(gamepad1);
        telemetry.addData("Status","Initialized");
        telemetry.update();
        specimenScore = 1000;
        bucketScore = 1500;
        climb = 2000;

        waitForStart();
        while(opModeIsActive()) {

            liftPower = controller.left_trigger.getTriggerValue() - controller.right_trigger.getTriggerValue();
            expectedManualLiftPos += (int)(-liftPower * 25);
            lift.setNewTargetPosition((expectedManualLiftPos));
            lift.setPower(liftPower);

//            if (controller.x.onPress())
//            {
//                lift.setPosition(specimenScore);
//            }else if(controller.y.onPress())
//            {
//                lift.setPosition(bucketScore);
//            }else if(gamepad1.dpad_up)
//            {
//                lift.setPosition(climb);
//            }else if(controller.a.onPress())
//            {
//                lift.setPosition(0);
//            }
//            telemetry.addData("Expected Target Pos", expectedManualLiftPos);
            telemetry.addData(" Lift Pos", lift.getPosition());

            telemetry.update();
            controller.update();
        }
    }

}




