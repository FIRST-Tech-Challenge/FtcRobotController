package org.firstinspires.ftc.teamcode.opmodes.test.DriveTests.Subsystems;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name="Servo Intake", group="TeleOp")
@Disabled
public class CRServoIntakeTest extends LinearOpMode {

    private CRServo con_servo;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {

  //      con_servo = hardwareMap.get(Servo.class, "intake");
        con_servo = hardwareMap.get(CRServo.class, "intake");


        waitForStart();
        while (opModeIsActive()) {
            //con_servo.setPosition(1.0);

            if (gamepad1.a) {
                con_servo.setPower(1);
            }
            if (gamepad1.b) {
                con_servo.setPower(-1);
            }
            if (gamepad1.x) {
                con_servo.setPower(0);
            }
            telemetry.update();
        }
        con_servo.setPower(0); // You have a potential to have a run after the loop, just shut it off when the opmode closes


    }
}