package org.firstinspires.ftc.teamcode;




import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp




public class NahuOpMode extends LinearOpMode {
    private DcMotor lMotor;
    private DcMotor rMotor;


    @Override
    public void runOpMode(){
        lMotor = hardwareMap.get(DcMotor.class, "lMotor");
        rMotor = hardwareMap.get(DcMotor.class, "rMotor");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();


        while (opModeIsActive()){


            // a button
            if (gamepad1.y){
                lMotor.setPower(-1);
                rMotor.setPower(1);
                sleep(1000);
                lMotor.setPower(0);
                rMotor.setPower(0);
            }


            // y button
            if (gamepad1.a) {
                lMotor.setPower(1);
                rMotor.setPower(-1);
                sleep(1000);
                lMotor.setPower(0);
                rMotor.setPower(0);
            }


            // b button
            else if (gamepad1.x) {
                lMotor.setPower(1);
                rMotor.setPower(1);
                sleep(500);
                lMotor.setPower(0);
                rMotor.setPower(0);
            }


            // x button
            else if (gamepad1.b) {
                lMotor.setPower(-1);
                rMotor.setPower(-1);
                sleep(500);
                lMotor.setPower(0);
                rMotor.setPower(0);
            }

            telemetry.addData("Status", "Running");
            telemetry.update();
        }


        lMotor.setPower(0);
        rMotor.setPower(0);
    }
}

