package org.firstinspires.ftc.teamcode.teleop;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

public class LiftTester {
    private Lift lift;
    private GamepadEvents controller;
    public void runOpMode(){
        lift = new Lift(hardwareMap);
        telemetry.addData("Status","Initialized");
        telemetry.update();
        while(linearOpMode.opModeIsActive()){
            double liftPower = controller.right_trigger.getTriggerValue();
            lift.moveLift(liftPower);

            telemetry.addData("Lift Power", liftPower);
            telemetry.addData("Lift Position", lift.getPosition());
            telemetry.update();
        }
    }

}
