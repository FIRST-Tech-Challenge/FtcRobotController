package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class RunAMotorConstantPower extends LinearOpMode{

    // Declare Motor
    private DcMotor myMotor = null;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized"); // Standard Telemetry
        telemetry.update();

        // Hardware map: Let's the robot know the name of the motor so that
        // We can reference it later in the config file
        myMotor = hardwareMap.get(DcMotor.class, "myMotor");

        // Set the direction of the motor to spin FORWARD
        myMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        waitForStart();

        // runs until the end of the match (until the driver presses STOP
        while(opModeIsActive()){
            // If the gamepad is moved forward above a certain value, set the motor to half power
            if(-gamepad1.left_stick_y > 0.5){
                myMotor.setPower(0.5);
            }
            else{
                //Otherwise switch it off
                myMotor.setPower(0.0);
            }
            // Telemetry so that we can see the value of the motor
            telemetry.addData("Left Stick", "Forward/Backwards Power (%.2f)", -gamepad1.left_stick_y);
            telemetry.update();
        }
    }
}
