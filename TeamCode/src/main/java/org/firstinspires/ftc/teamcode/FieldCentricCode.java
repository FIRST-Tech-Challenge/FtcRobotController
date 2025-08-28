package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class FieldCentricCode extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor FLM = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor BLM = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor FRM = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor BRM = hardwareMap.dcMotor.get("backRightMotor");

        FLM.setDirection(DcMotorSimple.Direction.REVERSE);
        BLM.setDirection(DcMotorSimple.Direction.REVERSE);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.
        )


        )
    }
}
