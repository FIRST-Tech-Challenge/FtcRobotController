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

                frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

                frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                leftDrive.setTargetPosition(100);
                rightDrive.set
        }




}
