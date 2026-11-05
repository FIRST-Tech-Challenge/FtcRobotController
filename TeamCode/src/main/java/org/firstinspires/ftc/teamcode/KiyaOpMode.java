package org.firstinspires.ftc.teamcode;




import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;




@TeleOp




public class KiyaOpMode extends LinearOpMode {


    private DcMotor lMotor;
    private DcMotor rMotor;


    @Override
    public void runOpMode(){
        lMotor = hardwareMap.get(DcMotor.class, "lMotor");
        rMotor = hardwareMap.get(DcMotor.class, "rMotor");


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        double tgtPower=0;
        while (opModeIsActive()){


            tgtPower = -this.gamepad1.left_stick_y;
            lMotor.setPower(tgtPower);
            rMotor.setPower(-1*tgtPower);
            telemetry.addData("target Power", tgtPower);
            telemetry.addData("Left Motor Power", lMotor.getPower());
            telemetry.addData("Right Motor Power", rMotor.getPower());
            telemetry.addData("Status", "Running");
            telemetry.update();


        }
        lMotor.setPower(0);
        rMotor.setPower(0);


    }




}



