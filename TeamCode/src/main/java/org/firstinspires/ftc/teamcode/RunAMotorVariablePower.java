package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp
@Disabled
public class RunAMotorVariablePower extends LinearOpMode {

    // Declare Motors
    private DcMotor myMotor = null;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        myMotor = hardwareMap.get(DcMotor.class, "myMotor");
        myMotor.setDirection(DcMotor.Direction.FORWARD);
        waitForStart();

        while(opModeIsActive()){
            double myMotorPower;

            double drivePower = -gamepad1.left_stick_y; //instead of neg, set dir REVERSE
            myMotorPower = Range.clip(drivePower, -1.0, 1.0);

            myMotor.setPower(myMotorPower);
            telemetry.addData("myMotor", "Forward/Backwards Power (%.2f)", myMotorPower);
            telemetry.update();
        }
    }

    /**
     * Assignment:
     * Show the value of the Left D-Pad, the X Button, and the
     * Left Trigger on the phone screen
     *
     * Resources that might be helpful:
     * ReadFromGamepad.java
     * Gamepad class definition
     */

    @TeleOp
    public static class ReadFromGamepad_Template extends LinearOpMode{

        @Override
        public void runOpMode(){

            // runs until the end of the match (until the driver presses STOP)
            while(opModeIsActive()){

            }
        }
    }
}
