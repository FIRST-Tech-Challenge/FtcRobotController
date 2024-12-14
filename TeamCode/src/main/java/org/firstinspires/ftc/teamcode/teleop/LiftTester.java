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
    private double liftPower;
    public void runOpMode(){
        expectedManualLiftPos = 0;
        lift = new Lift(hardwareMap, "liftLeft", "liftRight");
        controller = new GamepadEvents(gamepad1);
        telemetry.addData("Status","Initialized");
        telemetry.update();
        waitForStart();
        while(opModeIsActive()){

            liftPower = controller.left_trigger.getTriggerValue() - controller.right_trigger.getTriggerValue();
//            expectedManualLiftPos += (int)(-liftPower * 25);
//            lift.setNewTargetPosition((expectedManualLiftPos));
            lift.moveLift(liftPower);
            telemetry.addData("Expected Target Pos", expectedManualLiftPos);
            telemetry.addData("Left Lift Power", lift.getLeftMotorPower());
            telemetry.addData("Right Lift Power", lift.geRightMotorPower());
            telemetry.addData("Left Lift Position", lift.getLeftPosition());
            telemetry.addData("Right Lift Position", lift.getRightPosition());
            telemetry.update();
            controller.update();
        }
    }

}
