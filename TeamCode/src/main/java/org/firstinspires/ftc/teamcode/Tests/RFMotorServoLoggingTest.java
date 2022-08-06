package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFDualServo;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFMotor;
import org.firstinspires.ftc.teamcode.Components.tseDepositor;
import org.firstinspires.ftc.teamcode.Components.Logger;

@Autonomous(name="RFMotorServoLoggingTest", preselectTeleOp = "OneGPTeleop")

public class RFMotorServoLoggingTest extends LinearOpMode{


    public void runOpMode() {
        waitForStart();
        LinearOpMode op = this;
        Logger logger = new Logger(this);
        RFMotor rfmotor = new RFMotor("motorRightFront", DcMotorSimple.Direction.FORWARD, this, DcMotor.RunMode.RUN_USING_ENCODER, true, DcMotor.ZeroPowerBehavior.BRAKE, logger);

        rfmotor.setPower(0.9);
        op.sleep(5000);
    }
}

