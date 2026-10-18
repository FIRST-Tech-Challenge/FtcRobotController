package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

    @TeleOp(name = "zero servo")
    public class Driver extends LinearOpMode {
        private Servo intakeGate;


        public void runOpMode() throws InterruptedException {
            intakeGate = hardwareMap.get(Servo.class, "intakeGate");


            waitForStart();

            while (opModeIsActive()){

                intakeGate.setPosition(.01);
            }


        }}