package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "Turret_Testing")
//@Disabled

public class Turret_Testing extends LinearOpMode {
    public void runOpMode() {

        telemetry.addData("Status", "Before new Robot");
        telemetry.update();
        Robot robot = new Robot(this, BasicChassis.ChassisType.ENCODER, false ,false,90);
        telemetry.addData("Status", "Done with new Robot");
        telemetry.update();

        telemetry.addData("Status", "Ready to go");
        telemetry.update();
        double diSTANCe=0;

        //Aiden - during competition day robot disconnected so we are trying this code
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }

        while (!isStopRequested()) {
            robot.updateTurretPositions();
            diSTANCe+=gamepad2.left_stick_y/10;
            robot.TurretAngleControlRotating(diSTANCe);
//            sleep(100);
            telemetry.addData("diSTANCe",diSTANCe);
            telemetry.addData("runtime",getRuntime());
            telemetry.update();

            robot.TurretManualRotation(gamepad2.right_stick_x);
            if(gamepad2.a){
                robot.FlipBasket(0);
            }
            if(gamepad2.b){
                robot.flipBasketArmHigh();
            }

//            sleep(100);

            robot.TurretManualExtension(gamepad2.right_trigger, gamepad2.left_trigger);


//            sleep(100);
        }

        idle();
    }
}