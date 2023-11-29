package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class MonPremierModeOp extends LinearOpMode {

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
        double varY = 0;
        double varX = 0;
        while (opModeIsActive()) {
            varY = this.gamepad1.left_stick_y;
            varX = this.gamepad1.left_stick_x;

            double motorSpeed  = Math.sqrt(varY*varY + varX*varX);

            if ((varY > deadZone && varX > deadZone))
            {
                motorA.setPower(motorSpeed - (-1*varX));
                analogWrite(M1_Right, 0);

                analogWrite(M2_Left, motorSpeed);
                analogWrite(M2_Right, 0);
            }

            // forward left
            else if ((varY > deadZone && varX < -deadZone))
            {
                analogWrite(M1_Left, motorSpeed);
                analogWrite(M1_Right, 0);

                analogWrite(M2_Left, motorSpeed - varX);
                analogWrite(M2_Right, 0);
            }

            // forward
            else if ((varY > deadZone && varX > -deadZone && varX < deadZone))
            {
                analogWrite(M1_Left, motorSpeed);
                analogWrite(M1_Right, 0);

                analogWrite(M2_Left, motorSpeed);
                analogWrite(M2_Right, 0);
            }

            //back right
            else if ((varY < -deadZone && varX > deadZone))
            {
                analogWrite(M1_Left, 0);
                analogWrite(M1_Right,motorSpeed - (-1*varX));

                analogWrite(M2_Left, 0);
                analogWrite(M2_Right, motorSpeed);
            }

            // back left
            else if ((varY < -deadZone && varX < -deadZone))
            {
                analogWrite(M1_Left, 0);
                analogWrite(M1_Right, motorSpeed);

                analogWrite(M2_Left, 0);
                analogWrite(M2_Right, motorSpeed - varX);
            }

            // backwards
            else if ((varY < -deadZone && varX > -deadZone && varX < deadZone))
            {
                analogWrite(M1_Left, 0);
                analogWrite(M1_Right, motorSpeed);

                analogWrite(M2_Left, 0);
                analogWrite(M2_Right, motorSpeed);
            }
            else
            {
                analogWrite(M1_Left, 0);
                analogWrite(M1_Right, 0);

                analogWrite(M2_Left, 0);
                analogWrite(M2_Right, 0);
            }

            motorA.setPower(tgtPowerA);
            motorB.setPower(tgtPowerB);

            telemetry.addData("Target Power A", tgtPowerA);
            telemetry.addData("Target Power B", tgtPowerB);
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }

}
