package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
public class Servotesting extends LinearOpMode {
    private CRServo Right;
    private CRServo Left;
    private DcMotor Crain;
    private DcMotor Spin;


    public void runOpMode() throws InterruptedException {
        Right = hardwareMap.get(CRServo.class,"Rights");
        Left = hardwareMap.get(CRServo.class, "Lefts");
        Crain = hardwareMap.get(DcMotor.class, "Crain");
        Spin = hardwareMap.get(DcMotor.class, "Spin");

        waitForStart();
        while (opModeIsActive()) {

            boolean pickup;
            boolean dropoff;
            double crainpower;
            boolean turning;

            crainpower = gamepad2.left_stick_y;
            pickup = gamepad2.x;
            dropoff = gamepad2.y;
            turning = gamepad2.b;


            Crain.setPower(crainpower);

            if (pickup==true) {
                Right.setPower(1);
                Left.setPower(-1);
            }

            if (dropoff==true){
                Right.setPower(-1);
                Left.setPower(1);

            }

            if (dropoff==false && pickup==false){
                Right.setPower(0);
                Left.setPower(0);

            }


            if (turning){
                Spin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Crain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Spin.setTargetPosition(0);
                Crain.setTargetPosition(0);
                Spin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Crain.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Spin.setPower(1);
                Crain.setPower(1);
                while(Spin.isBusy()){

                }
                Crain.setTargetPosition(1);
                Crain.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Crain.setPower(1);
                while (Crain.isBusy()){

                }
            }

        }


    }
}