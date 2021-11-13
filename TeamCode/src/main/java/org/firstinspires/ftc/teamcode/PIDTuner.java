package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.ArrayList;

/*
this opmode should tune the strafing PID's using the Ziegler-Nichols method

 */
@TeleOp(name="PID Tuner")
//@Disabled
public class PIDTuner extends LinearOpMode {
    /*declare OpMode members, initialize some classes*/
    Hardware robot          = new Hardware();
    ElapsedTime runtime     = new ElapsedTime();
    PIDController pid       = new PIDController(0,0,0);

    @Override
    public void runOpMode() {
        // declare some variables if needed

        //values i should get from this test
        //{ Kp, Ti, Td }
        //double[] strafePidResults = new double[3];
        //double[] rotatePidResults = new double[3];

        double Ku = .003;
        double t1,t2=0; // times
        int c1, c2=0;    //previous encoder count, and current count
        double dCdT;  //deriv of c with respect to t
        double Tu = 0;      //period of oscillation, seconds

        double scaleFactor = .001;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        //PID config
        pid.reset();
        pid.setOutputRange(-1,1);
        pid.setTolerance(1);
        pid.setSetpoint(100);
        pid.enable(runtime.seconds());


        //button locks
        boolean upCurPressed, upPrevPressed = false,
                downCurPressed, downPrevPressed = false,
                leftCurPressed, leftPrevPressed = false,
                rightCurPressed, rightPrevPressed = false;
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

            Ku seems to be 0.00767
            Tu seems to be 0.098 or 0.0 sooo
                                    Kp      Ti      Td
            PID                 |.004602| 0.048 | 0.012
            PI                  |.003452| 0.08  |
            P                   |.003835|       |
            PD                  |.006136|       | 0.012
            some overshoot      |.002557| 0.048 | 0.032
            no overshoot        |.001534| 0.048 | 0.032
            pessen integration  |.005369| 0.0384| 0.0144

                                    Kp      Ki      Kd
            PID                 |.004602|.095875| 0.000055
            PI                  |.003452|0.04315|
            P                   |.003835|       |
            PD                  |.006136|       | 0.000074
            some overshoot      |.002557|.053271| 0.000082
            no overshoot        |.001534|.031958| 0.000049
            pessen integration  |.005369|.139818| 0.000077

            standard PID, pessen integration seem especially promising
             */

            // do the ziegler-nichols method for the strafing PID

            //allow K to be adjusted with buttons
            upCurPressed = gamepad1.dpad_up;
            if (upCurPressed && !upPrevPressed ){
                Ku += scaleFactor;
            }
            upPrevPressed=upCurPressed;

            downCurPressed = gamepad1.dpad_down;
            if (downCurPressed && !downPrevPressed){
                Ku -= scaleFactor;
            }
            downPrevPressed=downCurPressed;

            leftCurPressed = gamepad1.dpad_left;
            if (leftCurPressed && !leftPrevPressed){
                scaleFactor /= 10;
            }
            leftPrevPressed=leftCurPressed;

            rightCurPressed = gamepad1.dpad_right;
            if (rightCurPressed && !rightPrevPressed){
                scaleFactor *= 10;
            }
            rightPrevPressed=rightCurPressed;

            //update pid coefficients
            pid.setPID(Ku,0,0);

            //find the change in encoder ticks (derivative), use that to find the time difference between max's
            t1 = t2;
            t2 = runtime.seconds();
            double dT = t2 - t1; //change in time

            c1 = c2;
            c2 = robot.driveFrontRight.getCurrentPosition();
            double dC = c2 - c1; //change in count

            dCdT = dC/dT; //change in encoder count with respect to time

            //run PID and power front right motor accordingly
            double power;
            if (gamepad1.a) pid.setSetpoint(10);
            else pid.setSetpoint(1000);
            power = pid.performPID(robot.driveFrontRight.getCurrentPosition(), runtime.seconds());
            robot.driveFrontRight.setPower(power);


            //telemetry
            telemetry.addData("current PID output: ", power);
            telemetry.addData("scale factor: ", scaleFactor);
            telemetry.addLine("---====data====---");
            telemetry.addData("time: ", t2);
            telemetry.addData("dT: ", dT);
            telemetry.addData("count: ", c2);
            telemetry.addData("dC: ", dC);
            telemetry.addData("dC/dT: ", dCdT);
            telemetry.addLine("---====results====---");
            telemetry.addData("Ku = ", Ku);
            telemetry.addData("Tu = ", Tu);
            telemetry.update();
            //if they press b, record data for a few seconds, then try to calculate Tu
            //this just assumes that Ku is accurate
            if (gamepad1.b) Tu = calcTu();

        }

    }

    //attempt to calculate Tu
    private double calcTu()
    {
        double startTime;
        double power;
        double Tu;
        ArrayList<Double> timeList = new ArrayList<>();
        //ArrayList<Double> powerList = new ArrayList<>();
        ArrayList<Integer> countList = new ArrayList<>();
        timeList.add(0.0);
        //powerList.add(0.0);
        countList.add(0);
        ArrayList<Double> timeAtMaxValues = new ArrayList<>();
        ArrayList<Double> TuList = new ArrayList<>();


        double oscillationTotal = 0;
        int size;
        double maxTu;
        double minTu;

        telemetry.update();
        //stop motor, reset encoders, then have it move to encoder count of 1000 and record data as is oscillates

        //stop motor and reset encoders
        robot.driveFrontRight.setPower(0);
        robot.driveFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //reset PID
        pid.reset();
        pid.setOutputRange(-1,1);
        pid.setTolerance(1);
        pid.setSetpoint(1000);
        startTime = runtime.seconds();
        pid.enable(startTime); //Now we're on the clock

        //record data for 4 seconds
        while (runtime.seconds() - startTime <= 4.0) {
            //run pid
            power = pid.performPID(robot.driveFrontRight.getCurrentPosition(), runtime.seconds());
            robot.driveFrontRight.setPower(power);

            //record results
            countList.add(robot.driveFrontRight.getCurrentPosition() - 1000);
            timeList.add(runtime.seconds() - startTime);
            //powerList.add(power);
        }
        robot.driveFrontRight.setPower(0);

        //analyze data somehow

        //find maxes
        for (int i = 1; i < timeList.size()-1; i++) { //parse every data point
            //if count is greater than previous count, and greater than next count, its a maximum, so save the time
            if (countList.get(i-1) <= countList.get(i) && countList.get(i+1) < countList.get(i)) {
                timeAtMaxValues.add(timeList.get(i));
            }
        }
        //scan maxes, find the average period of oscillation, only care about the second half of the dataset
        size = timeAtMaxValues.size();
        maxTu = timeAtMaxValues.get(size/2);
        minTu = timeAtMaxValues.get(size/2);
        for (int i = size/2; i < size; i++) {
            Tu = timeAtMaxValues.get(i) - timeAtMaxValues.get(i-1);
            if (maxTu < Tu) maxTu = Tu;
            if (minTu > Tu) minTu = Tu;
            oscillationTotal += Tu;
            TuList.add(Tu);
        }
        Tu = oscillationTotal/size; //might want to use minTu instead in the future

        //just for shits and giggles, make sure that the second half has a fairly constant period of oscillation
        for (int i = 1; i < TuList.size(); i++) {
            //if percent change in t is more than 5%, it's unstable
            if ( Math.abs(((TuList.get(i) - TuList.get(i-1))/TuList.get(i-1)) * 100.0) > 5 ) {
                telemetry.addLine("oscillation is unstable");
                telemetry.addData("min Tu: ", minTu);
                telemetry.addData("max Tu: ", maxTu);
                telemetry.update();

                sleep(2000);

                break;
            }
        }

        return Tu;
    }
}
