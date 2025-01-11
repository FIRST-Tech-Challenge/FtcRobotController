package org.firstinspires.ftc.masters.test;

import static org.firstinspires.ftc.masters.components.Intake.INTAKE_POWER;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.masters.components.ITDCons;
import org.firstinspires.ftc.masters.components.Init;
import org.firstinspires.ftc.masters.components.Intake;
import org.firstinspires.ftc.masters.components.Outake;

@Config // Enables FTC Dashboard
@TeleOp(name = "Intake Test")
public class intakeTest extends LinearOpMode {

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Servo intakeLeft =hardwareMap.get(Servo.class, "intakeLeft");
        Servo intakeRight = hardwareMap.get(Servo.class, "intakeRight");

        DcMotor  intake = hardwareMap.dcMotor.get("intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

       // Init init = new Init(hardwareMap);
       // Intake intake = new Intake(init, telemetry);

        telemetry.update();

        intakeRight.setPosition(ITDCons.intakeInitRight);
        intakeLeft.setPosition(ITDCons.intakeInitLeft);

        waitForStart();

        while (opModeIsActive()) {

        if (gamepad1.a){
            intakeLeft.setPosition(ITDCons.dropLeft);
            intakeRight.setPosition(ITDCons.dropRight);
        }
        if (gamepad1.b){
            intakeRight.setPosition(ITDCons.intakeInitRight);
            intakeLeft.setPosition(ITDCons.intakeInitLeft);
        }

        if (gamepad1.x){
            intake.setPower(INTAKE_POWER);
        }

        if (gamepad1.y){
            intake.setPower(0);
        }


        }
    }
}

