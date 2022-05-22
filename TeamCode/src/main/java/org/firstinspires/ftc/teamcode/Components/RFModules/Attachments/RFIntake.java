package org.firstinspires.ftc.teamcode.Components.RFModules.Attachments;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFDualServo;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFMotor;

public class RFIntake extends RFMotor {
    private RFMotor intake;

    LinearOpMode op;

    public RFIntake(String motorName, DcMotorSimple.Direction motorDirection, LinearOpMode opMode, boolean resetPos, DcMotor.ZeroPowerBehavior zeroBehavior) {
        super(motorName, motorDirection, opMode, RUN_USING_ENCODER, resetPos, zeroBehavior);

        intake = new RFMotor(motorName, motorDirection, opMode, RUN_USING_ENCODER, resetPos, zeroBehavior);

        op = opMode;

    }
}
