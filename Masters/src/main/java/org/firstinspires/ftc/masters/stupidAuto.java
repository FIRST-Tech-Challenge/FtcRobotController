package org.firstinspires.ftc.masters;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.masters.components.Init;
import org.firstinspires.ftc.masters.components.Outake;

@Config // Enables FTC Dashboard
@Autonomous(name = "Stupid")
public class stupidAuto extends LinearOpMode {

    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private DcMotor leftRearMotor;
    private DcMotor rightRearMotor;

    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Init init = new Init(hardwareMap);
        Outake outake = new Outake(init, telemetry);

        telemetry.update();

        this.leftFrontMotor = init.getLeftFrontMotor();
        this.rightFrontMotor = init.getRightFrontMotor();
        this.leftRearMotor = init.getLeftRearMotor();
        this.rightRearMotor = init.getRightRearMotor();

        waitForStart();


            leftFrontMotor.setPower(-.5);
            rightFrontMotor.setPower(-.5);
            leftRearMotor.setPower(-.5);
            rightRearMotor.setPower(-.5);
            sleep(1500);
            leftFrontMotor.setPower(0);
            rightFrontMotor.setPower(0);
            leftRearMotor.setPower(0);
            rightRearMotor.setPower(0);


    }
}

