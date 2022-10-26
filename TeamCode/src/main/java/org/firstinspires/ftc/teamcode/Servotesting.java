package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
public class Servotesting extends LinearOpMode {
    private Servo Right;
    private Servo Left;
    private DcMotor Crain;


    public void runOpMode() throws InterruptedException {
        Right = hardwareMap.get(Servo.class,"Rights");
        Left = hardwareMap.get(Servo.class, "Lefts");
        Crain = hardwareMap.get(DcMotor.class, "Crain");

        waitForStart();
        while (opModeIsActive()) {

            boolean pickup;
            boolean dropoff;
            double crainpower;

            crainpower = gamepad2.left_stick_y;
            pickup = gamepad2.x;
            dropoff = gamepad2.y;

            Crain.setPower(crainpower);

            if (pickup) {
                Right.setPosition(1);
                Left.setPosition(0);
            }
            if (dropoff){
                Right.setPosition(0);
                Left.setPosition(1);
            }
        }


    }
}