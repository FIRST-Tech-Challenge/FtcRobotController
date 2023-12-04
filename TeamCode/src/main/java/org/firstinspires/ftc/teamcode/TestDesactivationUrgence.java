package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class TestDesactivationUrgence extends LinearOpMode {

    private DcMotorEx motorA;
    private DcMotorEx motorB;



    @Override
    public void runOpMode() {
        motorA = hardwareMap.get(DcMotorEx.class, "moteur1");
        motorB = hardwareMap.get(DcMotorEx.class, "moteur2");

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
            if (this.gamepad1.dpad_up) { // test mode urgence -> coupe les moteurs
                while (this.gamepad1.dpad_up) {
                    motorA.setMotorDisable();
                    motorB.setMotorDisable();
                }
                }



            varY1 = this.gamepad1.left_stick_y;
            varY2 = this.gamepad1.right_stick_y;

            motorA.setPower(varY1);
            motorB.setPower(-varY2);


            telemetry.addData("Target Power A", varY1);
            telemetry.addData("Target Power B", -varY2);
            telemetry.addData("Power ON ?", motorA.isMotorEnabled());
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }

}
