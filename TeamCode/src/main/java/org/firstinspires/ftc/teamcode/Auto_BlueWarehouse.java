package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class Auto_BlueWarehouse extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    public static double encoderMethod(double distance){
//      384.5(PPM) = ~50cm = ~20in
//      7.9(PPM) = 1cm
        double ppm = distance * 7.9;
        return ppm;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        DcMotor duckWheel = hardwareMap.get(DcMotor.class, "duckWheel");
        DcMotorEx frontL  = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx frontR = hardwareMap.get(DcMotorEx.class, "rightFront");
        DcMotorEx backL  = hardwareMap.get(DcMotorEx.class, "leftRear");
        DcMotorEx backR = hardwareMap.get(DcMotorEx.class, "rightRear");
        DcMotorEx extender = hardwareMap.get(DcMotorEx.class, "extender");
        DcMotorEx arm = hardwareMap.get(DcMotorEx.class, "arm");

        // Reset Encoder
        frontL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


        /* Sets the motors to run using encoders. */

        /* Most robots need the motor on one side to be reversed to drive forward.
         * Reverse the motor that runs backwards when connected directly to the battery. */
        frontL.setDirection(DcMotorEx.Direction.FORWARD);
        backL.setDirection(DcMotorEx.Direction.FORWARD);
        frontR.setDirection(DcMotorEx.Direction.REVERSE);
        backR.setDirection(DcMotorEx.Direction.REVERSE);
//      Full revolution 384.5(PPR) = ~50cm = ~20in

        int power = 100;
        double distance = 0.0;


        waitForStart();
//      Update telemetry status to show that it is running
        telemetry.addData("Status", "Running");
        telemetry.update();

        frontL.setTargetPosition((int) Math.round(encoderMethod(distance)));
        frontR.setTargetPosition((int) Math.round(encoderMethod(distance)));
        backL.setTargetPosition((int) Math.round(encoderMethod(distance)));
        backR.setTargetPosition((int) Math.round(encoderMethod(distance)));

        frontL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        frontR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        frontL.setVelocity(power);
        frontR.setVelocity(power);
        backL.setVelocity(power);
        backR.setVelocity(power);



        while (frontL.isBusy() || frontR.isBusy() || backL.isBusy() || backR.isBusy()) {
            telemetry.addData("Status", "Waiting for Motors");
            telemetry.addData("Motors", "frontL Position: %d", frontL.getCurrentPosition());
            telemetry.addData("Motors", "frontR Position: %d", frontR.getCurrentPosition());
            telemetry.addData("Motors", "backL Position: %d", backL.getCurrentPosition());
            telemetry.addData("Motors", "backR Position: %d", backR.getCurrentPosition());
            telemetry.update();
        }
//      Stop the Autonomous Mode after we finish parking
    }
}
