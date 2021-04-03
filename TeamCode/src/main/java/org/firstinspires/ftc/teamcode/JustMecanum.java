

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Just Mecanum", group="Linear Opmode")

public class JustMecanum extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();

    /*
    private DcMotor rightManipulator = null;
    //private DcMotor leftManipulator = null;
    private final int FL=0;
    private final int FR=1;
    private final int BL =2;
    private final int BR =3;
    private final double SLOW=0.4;
    private final double MAX_SPEED=2800;
    Servo capstone;
    Servo foundationRight;
    Servo foundationLeft;
    double capstonePosition=0;
    private double mSpeed=1.0;

     */

    private MecanumDrive mecanumDrive = new MecanumDrive();

    @Override
    public void runOpMode() {
        mecanumDrive.init(hardwareMap, telemetry, this);


        //capstone = hardwareMap.get(Servo.class, "capstone");
        //foundationRight = hardwareMap.get(Servo.class, "foundationRight");
        //foundationLeft = hardwareMap.get(Servo.class, "foundationLeft");



        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            gamepad1.setJoystickDeadzone((float)0.2);
            double speed = 1;

             speed = (gamepad1.right_trigger * 0.6) + 0.4;
            double fwd = gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rot= gamepad1.right_stick_x;

            fwd = fwd * speed;
            strafe =strafe * speed * 1.6;
            if (strafe > 1) {
                strafe = 1;
            } else if (strafe < -1) {
                strafe = -1;
            }
            rot = rot * speed;
            mecanumDrive.setMotors(strafe,fwd,rot, 1);

            boolean pull = gamepad2.x;
            boolean push = gamepad2.b;

            if (gamepad2.right_bumper) {
                //mecanumDrive.setMotors(0,0,0,1);
                sleep(100);
                sleep(100);

            } else {
            }




            mecanumDrive.tickSleep();
            telemetry.addData("Left/Right Stick", "LX (%.2f), LY (%.2f), RX (%.2f), RY (%.2f)", gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_y);

            telemetry.update();
        }

    }

}
