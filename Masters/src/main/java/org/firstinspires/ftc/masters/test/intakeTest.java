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

@Config // Enables FTC Dashboard
@TeleOp(name = "Intake Test")
public class intakeTest extends LinearOpMode {

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Servo intakeLeft =hardwareMap.get(Servo.class, "intakeLeft");
        Servo intakeRight = hardwareMap.get(Servo.class, "intakeRight");
        Servo gate = hardwareMap.get(Servo.class,"gateServo");

        DcMotor  intake = hardwareMap.dcMotor.get("intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

       // Init init = new Init(hardwareMap);
       // Intake intake = new Intake(init, telemetry);

        telemetry.update();

//        intakeRight.setPosition(ITDCons.intakeInitRight);
       // intakeRight.setPosition(0.5);

        waitForStart();

        while (opModeIsActive()) {

        if (gamepad1.a){
            intakeLeft.setPosition(ITDCons.intakeArmDrop);
            intakeRight.setPosition(ITDCons.intakeChainDrop);
        }
        if (gamepad1.b){
            intakeLeft.setPosition(ITDCons.intakeArmTransfer);
            intakeRight.setPosition(ITDCons.intakeChainTransfer);
        }

        if (gamepad1.x){
            intake.setPower(INTAKE_POWER);
        }

        if (gamepad1.y){
            intake.setPower(0);
        }

        if (gamepad1.dpad_up){
            gate.setPosition(ITDCons.gateClose);
        }

        if (gamepad1.dpad_down){
            gate.setPosition(ITDCons.gateOpen);
        }


        }
    }
}

