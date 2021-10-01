package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="[ACTIVE] Motor Test", group="K9bot")
public class Motortest extends LinearOpMode{

    /* Declare OpMode members. */
    MecanumHardware   robot           = new MecanumHardware();


    /* ---------------------------------------------------------------------------------------------
    double          armPosition     = robot.ARM_HOME;                   // Servo safe position
    double          clawPosition    = robot.CLAW_HOME;                  // Servo safe position
    final double    CLAW_SPEED      = 0.01 ;                            // sets rate to move servo
    final double    ARM_SPEED       = 0.01 ;                            // sets rate to move servo
    ---------------------------------------------------------------------------------------------*/



    boolean changed = false, on = false; //Outside of loop()
    @Override
    public void runOpMode() {
        double motor0;
        double motor1;
        double motor2;
        double motor3;


        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
            motor0 = -gamepad1.left_stick_y;
            motor1 = gamepad1.left_stick_x;
            motor2 = gamepad1.right_stick_x;
            motor3 = -gamepad1.right_stick_y;


            //--------------------------------------------------------------------------------------
            //--------------------------------------------------------------------------------------
            //--------------------------------------------------------------------------------------
            // Driving using IF statements to make sure that we are not conflicting

            // Drive and turn
            if ((motor0 < -0.2 || motor0 > 0.2) || (motor1 < -0.2 || motor1 > 0.2)) {
                robot.leftFront.setPower(motor0);
                robot.leftBack.setPower(motor1);

                //Drive and Strafe
            } else if ((motor2 < -0.2 || motor2 > 0.2) || (motor3 < -0.2 || motor3 > 0.2)) {
                robot.rightBack.setPower(motor2);
                robot.rightFront.setPower(motor3);

                //Stop
            } else {
                robot.leftFront.setPower(0);
                robot.leftBack.setPower(0);
                robot.rightFront.setPower(0);
                robot.rightBack.setPower(0);
            }

            // Pause for 10 mS each cycle = update 100 times a second.
            sleep(10);
        }
    }
}


