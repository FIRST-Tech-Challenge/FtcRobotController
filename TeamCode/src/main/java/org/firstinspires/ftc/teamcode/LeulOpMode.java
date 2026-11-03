package org.firstinspires.ftc.teamcode;




import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp




public class LeulOpMode extends LinearOpMode {
    private DcMotor lMotor;
    private DcMotor rMotor;


    @Override
    public void runOpMode(){
        lMotor = hardwareMap.get(DcMotor.class, "lMotor");
        rMotor = hardwareMap.get(DcMotor.class, "rMotor");


        lMotor.setDirection(DcMotor.Direction.FORWARD);
        rMotor.setDirection(DcMotor.Direction.REVERSE);


        waitForStart();


        while (opModeIsActive()){


            // a button
            if (gamepad1.a){
                lMotor.setPower(-1);
                rMotor.setPower(-1);
            }


            // y button
            else if (gamepad1.y) {
                lMotor.setPower(1);
                rMotor.setPower(1);
            }


            // b button
            else if (gamepad1.b) {
                lMotor.setPower(1);
                rMotor.setPower(-1);
            }


            // x button
            else if (gamepad1.x) {
                lMotor.setPower(-1);
                rMotor.setPower(1);
            }


            else{
                lMotor.setPower(0);
                rMotor.setPower(0);
            }


        }


        lMotor.setPower(0);
        rMotor.setPower(0);
    }
}

