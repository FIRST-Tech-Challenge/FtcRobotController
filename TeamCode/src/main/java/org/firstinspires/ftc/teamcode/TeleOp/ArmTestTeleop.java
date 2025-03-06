package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DeliveryFunctions;
import org.firstinspires.ftc.teamcode.IntakeFunctions;

@Disabled
@TeleOp(name="Arm Test Teleop", group="A")
public class ArmTestTeleop extends LinearOpMode {
    private DeliveryFunctions deliveryFunctions;
    private IntakeFunctions intakeFunctions;
    private int leftMotorPosition;
    private int rightMotorPosition;
    private final double DEADZONE = 0.05;


    public void runOpMode() {
        deliveryFunctions = new DeliveryFunctions(this, false);
        intakeFunctions = new IntakeFunctions(this);
        waitForStart();

        while(opModeIsActive()){

            telemetry.addData("Servo Position: ", deliveryFunctions.getWristPosition());


            if(Math.abs(gamepad2.left_stick_y) > DEADZONE){

                deliveryFunctions.setSlidesPower(gamepad2.left_stick_y * 0.9);

            }

            if(Math.abs(gamepad2.right_stick_y) > DEADZONE){

                deliveryFunctions.setSlidesPower(gamepad2.right_stick_y / 3);

            }

            if(Math.abs(gamepad2.left_stick_y) < DEADZONE && Math.abs(gamepad2.right_stick_y) < DEADZONE){
                deliveryFunctions.setSlidesPower(0);
            }



            if(gamepad2.right_trigger > DEADZONE){
                intakeFunctions.RunIntakeMotor(gamepad2.right_trigger);
            } else{
                intakeFunctions.RunIntakeMotor(0);
            }


            telemetry.addData("Left Motor Position: ",deliveryFunctions.getMotorPositionByIndex(0));
            telemetry.addData("Right Motor Position: ",deliveryFunctions.getMotorPositionByIndex(1));

            //telemetry.addData("Target Motor Position ",deliveryFunctions.getMotorTargetPosition());

            telemetry.update();

        }
    }
}
