package org.firstinspires.ftc.teamcode.Chloe;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class chloeAuto extends LinearOpMode {

    public void proportionalMove(int inches) {
        DcMotor fLeft = hardwareMap.dcMotor.get("fLeft");
        DcMotor bLeft = hardwareMap.dcMotor.get("bLeft");
        DcMotor fRight = hardwareMap.dcMotor.get("fRight");
        DcMotor bRight = hardwareMap.dcMotor.get("bRight");

        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        fRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bRight.setDirection(DcMotorSimple.Direction.FORWARD);

        fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       // fLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // start of PID part (P)
        double ticksPerInch = (537.7 / 1.4) / 11.87373601322835;
        double error = ticksPerInch * inches;
        double P = 0.0008;

        telemetry.addData("error", error);
        telemetry.update();

        while (opModeIsActive() && Math.abs(error) > 100) {
            error = ticksPerInch * inches - fLeft.getCurrentPosition();

            telemetry.addData("motor power", error * P);
            telemetry.addData("still moving ", error);
            telemetry.update();
             //if(inches > 0) {
                fLeft.setPower(error * P);
                bLeft.setPower(error * P);
                fRight.setPower(error * P);
                bRight.setPower(error * P);
           /* } else {
                fLeft.setPower(error * -P);
                bLeft.setPower(error * -P);
                fRight.setPower(error * -P);
                bRight.setPower(error * -P);
            }*/
        }
        telemetry.addLine("done moving "+ inches);
        telemetry.update();

        if (fLeft.getCurrentPosition() > fLeft.getTargetPosition()) {
            telemetry.addLine("help-");
            telemetry.update();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException{

        waitForStart();
        proportionalMove(12);
        proportionalMove(-12);
        //encoderMove(12); // moves forward 12 inches
        // encoderMove(-12); // moves backward 12 inches
    }

}





