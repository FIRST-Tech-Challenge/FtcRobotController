package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
this opmode should tune the strafing PID's using the Ziegler-Nichols method

 */

@TeleOp(name="PID Tuner")
public class PIDTuner extends LinearOpMode {
    /*declare OpMode members, initialize some classes*/
    Hardware robot          = new Hardware();
    ElapsedTime runtime     = new ElapsedTime();
    PIDController pid       = new PIDController(15,0,0);



    @Override
    public void runOpMode(){
        // declare some variables if needed

        //values i should get from this test
        //{ Kp, Ti, Td }
        double[] strafePidResults = new double[3];
        double[] rotatePidResults = new double[3];

        double Ku = 15;
        double t1=0,t2=0; // times
        int c1=0, c2=0;    //previous encoder count, and current count
        double d1=0,d2=0;  //previous deriv of c, and current deriv of c
        double m1=0,m2=0; //the time, in seconds, of the most recent max, and the max before that

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        pid.setSetpoint(0);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        //button locks
        boolean upCurPressed, upPrevPressed = false, downCurPressed, downPrevPressed = false;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /*
            ziegler-nichols procedure

            1. start with Kp = small number, set Ki and Kd to 0
            2. start increasing Kp until neutral stability (stable oscillation, neither converge to zero nor diverges from it)
            3. record critical (ultimate) gain Ku (Ku = Kp at neutral stability)
            4. also record the period of oscillation at that gain Tu in seconds
            5. lookup associate Kp, Ti, Td:
                                    Kp      Ti      Td
            PID                 | 0.6Ku | Tu/2  | Tu / 8
            PI                  | 0.45Ku| Tu/1.2|
            P                   | 0.5Ku |       |
            PD                  | 0.8Ku |       | Tu/8
            some overshoot      | Ku/3  | Tu/2  | Tu/3
            no overshoot        | 0.2Ku | Tu/2  | Tu/3
            pessen integration  | 0.7Ku | 2Tu/5 | 3Tu/20

            Kp = Kp
            Ki = Kp / Ti
            Kd = Kp * Td

             */

            // do the ziegler-nichols method for the strafing PID

            //allow K to be adjusted with buttons
            upCurPressed = gamepad1.dpad_up;
            if (upCurPressed && !upPrevPressed ){
                Ku += 1;
            }
            upPrevPressed=upCurPressed;

            downCurPressed = gamepad1.dpad_down;
            if (downCurPressed && !downPrevPressed){
                Ku -= 1;
            }
            downPrevPressed=downCurPressed;

            //update pid coefficients
            pid.setPID(Ku,0,0);


            //find the change in encoder ticks (derivative), use that to find the time difference between max's
            t1 = t2;
            t2 = runtime.seconds();
            double dT = t2 - t1; //change in time

            c1 = c2;
            c2 = robot.driveBackRight.getCurrentPosition();
            double dC = c2 - c1; //change in count

            d1 = d2;
            d2 = dC/dT; //change in encoder count with respect to time

            if (d2 == 0 && c2 != 0) {
                m1 = m2;
                m2 = t2;
            }

            //telemetry
            telemetry.addLine("---====data====---");
            telemetry.addData("time: ", t2);
            telemetry.addData("dT: ", dT);
            telemetry.addData("count: ", c2);
            telemetry.addData("dC: ", dC);
            telemetry.addLine("---====results====---");
            telemetry.addData("Ku = ", Ku);
            telemetry.addData("Tu = ", m2-m1);

            telemetry.update();

            //run PID and power front right motor accordingly
            robot.driveFrontRight.setPower(pid.performPID(c2));

        }

    }
}
