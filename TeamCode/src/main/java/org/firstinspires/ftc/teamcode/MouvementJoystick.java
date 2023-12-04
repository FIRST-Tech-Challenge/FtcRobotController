package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class MouvementJoystick extends LinearOpMode {

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

        double deadZone = 0.1;
        double varY = 0;
        double varX = 0;
        double varYpos = 0;
        double varXpos = 0;
        double speed = 0;

        while (opModeIsActive()) {
            varY = this.gamepad1.left_stick_y;
            varX = this.gamepad1.left_stick_x;

            varYpos = Math.abs(varY);
            varXpos = Math.abs(varX);
            speed = Math.sqrt(varXpos*varXpos+varYpos*varYpos);



            if(varY > 0) //Forward
            {
                tgtPowerA=varYpos;
                tgtPowerB=varYpos;

                if (varX < 0) {
                    tgtPowerA=tgtPowerA-varXpos;
                }
                else if (varX > 0) {
                    tgtPowerB=tgtPowerB-varXpos;
                }

            }
            else if(varY < 0) //Backward
            {
                tgtPowerA=-varYpos;
                tgtPowerB=-varYpos;

                if (varX < 0) {
                    tgtPowerA=tgtPowerA+varXpos;
                }
                else if (varX > 0) {
                    tgtPowerB=tgtPowerB+varXpos;
                }

            }
            //joyValue Between joyValueMidLower - joyValueMidUpper.
            //Need some range here, because joystick sometime not in  exact center.
            else if (varY==0)
            {
                tgtPowerA=0;
                tgtPowerB=0;
            }

            if (varX>0 && varY==0) {
                tgtPowerA=-varXpos;
                tgtPowerB=varXpos;
            }

            if (varX<0 && varY==0) {
                tgtPowerA=varXpos;
                tgtPowerB=-varXpos;
            }


            if (this.gamepad1.left_bumper) {
                motorA.setPower(tgtPowerA);
                motorB.setPower(-tgtPowerB);
            } else {
                motorA.setPower(tgtPowerA/2);
                motorB.setPower(-(tgtPowerB/2));
            }

            telemetry.addData("Target Power A", tgtPowerA);
            telemetry.addData("Target Power B", tgtPowerB);
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }

}
