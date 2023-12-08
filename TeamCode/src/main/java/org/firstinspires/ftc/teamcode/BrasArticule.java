package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class BrasArticule extends LinearOpMode {

    private DcMotorEx motorA;
    private DcMotorEx motorB;

    private DcMotorEx bras1;
    private DcMotorEx bras2;

    private Servo coude;

    private Servo mains;


    @Override
    public void runOpMode() {
        //motorA = hardwareMap.get(DcMotorEx.class, "moteur1");
        //motorB = hardwareMap.get(DcMotorEx.class, "moteur2");
        bras1 = hardwareMap.get(DcMotorEx.class, "bras1");
        bras2 = hardwareMap.get(DcMotorEx.class, "bras2");
        coude = hardwareMap.get(Servo.class, "coude");
        mains = hardwareMap.get(Servo.class, "mains");



        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        double coudeX = 0.5;
        int brasA = 0;
        int brasB = 0;
        double trigger = 0;
        double varRY = 0;

        bras1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bras2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while (opModeIsActive()) {
            varRY = this.gamepad1.right_stick_y;
            trigger = this.gamepad1.right_trigger;

            brasA = bras1.getCurrentPosition();
            brasB = bras2.getCurrentPosition();


            if (varRY < 0) {
                bras1.setPower(varRY/2);
                bras2.setPower(varRY/2);
            } else {
                bras1.setPower(varRY/4);
                bras2.setPower(varRY/4);
            }

            if (this.gamepad1.dpad_up) {
                coudeX += 0.002;
                if (coudeX > 1) {
                    coudeX = 1;
                }
            } else if (this.gamepad1.dpad_down) {

                coudeX -= 0.002;
                if (coudeX<0) {
                    coudeX = 0;
                }
            } else {
                coudeX = 0.5;
            }
            coude.setPosition(coudeX);

            mains.setPosition(trigger);

            telemetry.addData("Var Y", varRY);
            telemetry.addData("Coude", coudeX);
            telemetry.addData("M1", brasA);
            telemetry.addData("M2", brasB);
            telemetry.addData("Trigger", trigger);
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }

}
