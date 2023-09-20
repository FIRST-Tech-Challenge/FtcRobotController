package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "Gibson's op mode", group = "NWR-FTC")
public class Gibsonsopmode extends LinearOpMode{
    DcMotor motorLeft;
    DcMotor motorRight;
    DcMotor frontLeft;
    DcMotor frontRight;

    @Override
    public void runOpMode() {
        //Initialization code
        telemetry.addData("Seagulls... stop it now!!! ad sponsered by yourmom.com", 0);
        telemetry.addData("please open README.Scam with your IP address visibly on your screen!!!", 0);
        waitForStart();

        telemetry.addData("ad sponsered by yourmom.com", 0);
        telemetry.update();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; //y axis on left stick
            double X = -gamepad1.left_stick_x; //X axis on left stick


            if(y == 1){
                motorLeft.setPower(-1);
                motorRight.setPower(1);
            }
            else if(y == -1){
                motorLeft.setPower(1);
                motorRight.setPower(-1);
            }
            else if(y == 0){
                motorLeft.setPower(0);
                motorRight.setPower(0);
            }
            else if(X == 1){
                motorLeft.setPower(0.5);
                frontLeft.setPower(0.5);
                motorRight.setPower(0.5);
                frontRight.setPower(0.5);
            }
            else if(X == -1){
                motorLeft.setPower(-0.5);
                frontLeft.setPower(-0.5);
                motorRight.setPower(0.5);
                frontRight.setPower(0.5);
            }
            else {
                motorLeft.setPower(0);
                motorRight.setPower(0);
            }


        }
    }
}
