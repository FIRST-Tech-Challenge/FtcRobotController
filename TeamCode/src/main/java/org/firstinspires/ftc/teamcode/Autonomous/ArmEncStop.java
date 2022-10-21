package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Functions.EncoderMove;
import org.firstinspires.ftc.teamcode.Functions.Arm;
import org.firstinspires.ftc.teamcode.Functions.Collector;
import org.firstinspires.ftc.teamcode.Functions.EncoderMove;
import org.firstinspires.ftc.teamcode.Functions.Move;
import org.firstinspires.ftc.teamcode.Functions.MoveAutocorrect2;
import org.firstinspires.ftc.teamcode.Functions.Rotate;
import org.firstinspires.ftc.teamcode.Functions.RotationDetector;
import org.firstinspires.ftc.teamcode.Functions.VoltageReader;

@Autonomous(name = "ArmEncStop", group = "Concept")
public class ArmEncStop extends LinearOpMode {

    private DcMotorEx armMotorLeft, armMotorRight;

    @Override
    public void runOpMode(){


        armMotorLeft = hardwareMap.get(DcMotorEx.class, "_AML");
        armMotorRight = hardwareMap.get(DcMotorEx.class, "_AMR");

        armMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if(gamepad1.right_bumper && armMotorLeft.getCurrentPosition() < 0){
            armMotorLeft.setPower(1);
            armMotorRight.setPower(1);
        }
        else if(gamepad1.left_bumper && armMotorLeft.getCurrentPosition() > 0){
            armMotorLeft.setPower(-1);
            armMotorRight.setPower(-1);
        }
        else
        {
            armMotorLeft.setPower(0);
            armMotorRight.setPower(0);
        }
    }
}

