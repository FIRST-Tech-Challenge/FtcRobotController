package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class testmoteurmateo extends LinearOpMode {

    private DcMotor motorA;
    private DcMotor motorB;



    @Override
    public void runOpMode() {
        motorA = hardwareMap.get(DcMotor.class, "moteur1");
        motorB = hardwareMap.get(DcMotor.class, "moteur2");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        double tgtPowerA = 0;
        double tgtPowerB = 0;

        double deadZone = 0.2;
        double varY1 = 0;
        double varY2 = 0;
        double varX = 0;
        while (opModeIsActive()) {
            varY1 = this.gamepad1.left_stick_y;
            varY2 = this.gamepad1.right_stick_y;

            motorA.setPower(varY1);
            motorB.setPower(-varY2);


            telemetry.addData("Target Power A", varY1);
            telemetry.addData("Target Power B", -varY2);
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }

}
