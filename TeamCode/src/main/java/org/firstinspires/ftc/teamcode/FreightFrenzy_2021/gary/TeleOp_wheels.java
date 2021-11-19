package org.firstinspires.ftc.teamcode.FreightFrenzy_2021.gary;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;



@TeleOp(name="GWheels test", group="Linear Opmode")
public class TeleOp_wheels extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;


    double speed = 0.5;



    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        runtime.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (gamepad1.right_bumper == true) {
                while (gamepad1.right_bumper) {
                }
                speed += 0.05;
            }

            if (gamepad1.left_bumper == true) {
                while (gamepad1.left_bumper) {
                }
                speed -= 0.05;
            }


            double lF_P;
            double lB_P;
            double rF_P;
            double rB_P;

            double dRL = -gamepad1.left_stick_y;
            double drive = -gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;

            lF_P = Range.clip(speed * (drive + turn - dRL), -1.0, 1.0);
            rF_P = Range.clip(speed * (drive + turn + dRL), -1.0, 1.0);
            lB_P = Range.clip(speed * (drive - turn + dRL), -1.0, 1.0);
            rB_P = Range.clip(speed * (drive - turn - dRL), -1.0, 1.0);

            leftFront.setPower(lF_P);
            rightFront.setPower(rF_P);
            leftBack.setPower(lB_P);
            rightBack.setPower(rB_P);


            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("FrontMotors", "left (%.2f), right (%.2f)", lF_P, rF_P);
            telemetry.addData("BackMotors", "left (%.2f), right (%.2f)", lB_P, rB_P);
            telemetry.addData("Speed:", speed);
            telemetry.update();


        }
    }
}
