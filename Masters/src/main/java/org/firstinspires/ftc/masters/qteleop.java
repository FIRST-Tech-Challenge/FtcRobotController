package org.firstinspires.ftc.masters;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.masters.components.ControllerMap;
import org.firstinspires.ftc.masters.components.DriveTrain;
import org.firstinspires.ftc.masters.components.Intake;

@Config // Enables FTC Dashboard
@TeleOp(name = "Wheel Slip Test", group = "tests")
public class qteleop extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        DriveTrain driveTrain = new DriveTrain(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        while(true) {
            driveTrain.driveNoMultiplier(gamepad1, DriveTrain.RestrictTo.XYT);
            if (gamepad1.a){
                intake.Forward();
            }
            if (gamepad1.b){
                intake.Reverse();
            }
        }
    }
}
