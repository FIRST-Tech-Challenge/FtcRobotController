package org.firstinspires.ftc.teamcode.developingTestCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="Auton Test")
public class AutonTest extends LinearOpMode{

        @Override
        public void runOpMode() {
                DcMotor frontLeftMotor = hardwareMap.dcMotor.get("motorFL");
                DcMotor backLeftMotor = hardwareMap.dcMotor.get("motorBL");
                DcMotor frontRightMotor = hardwareMap.dcMotor.get("motorFR");
                DcMotor backRightMotor = hardwareMap.dcMotor.get("motorBR");

                frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

                frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                frontRightMotor.setTargetPosition(100);
                frontLeftMotor.setTargetPosition(100);
                backLeftMotor.setTargetPosition(100);
                backRightMotor.setTargetPosition(100);

                frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }




}
