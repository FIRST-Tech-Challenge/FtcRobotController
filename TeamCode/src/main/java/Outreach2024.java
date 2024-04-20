package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="Tank Tele-Op", group="Robot")

public class OutreachBot2024 extends OpMode{

   /* Declare OpMode members. */
   public DcMotor  frontLeft   = null;
   public DcMotor  frontRight = null;
   public DcMotor  backLeft = null;
   public DcMotor  backRight = null;

   public void init() {
       // Define and Initialize Motors
       frontLeft  = hardwareMap.get(DcMotor.class, "LF");
       backLeft  = hardwareMap.get(DcMotor.class, "LB");
       frontRight = hardwareMap.get(DcMotor.class, "RF");
       backRight  = hardwareMap.get(DcMotor.class, "RB");
       frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       frontLeft.setDirection(DcMotor.Direction.FORWARD);
       frontRight.setDirection(DcMotor.Direction.FORWARD);
       backLeft.setDirection(DcMotor.Direction.FORWARD);
       backRight.setDirection(DcMotor.Direction.FORWARD);
       frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



       telemetry.addData(">", "Robot Ready.  Press Play.");    //
   }

   public void init_loop() {
   }

   public void start() {
       telemetry.clear();
   }

   public void loop() {
       double left1y;
       double right1x;

       left1y = gamepad1.left_stick_y;
       right1x = gamepad1.right_stick_x;

       double leftPower = Math.min(Math.abs(left1y+right1x),1)*Math.signum(left1y+right1x);
       double rightPower = Math.min(Math.abs(left1y-right1x),1)*Math.signum(left1y-right1x);
       frontLeft.setPower(leftPower);
       frontRight.setPower(rightPower);
       backLeft.setPower(leftPower);
       backRight.setPower(rightPower);

       }

   public void stop() {
   }
}
