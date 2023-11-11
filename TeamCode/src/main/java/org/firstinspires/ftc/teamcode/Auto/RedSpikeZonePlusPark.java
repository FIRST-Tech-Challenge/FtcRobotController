package org.firstinspires.ftc.teamcode.Auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous (name="Spike Zone + Park")
public class RedSpikeZonePlusPark extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

    }

    @Autonomous(name = "RedObjectDistance")
    public class RedSpikeZonePark extends LinearOpMode {

        private DistanceSensor distanceSensor;
        private DcMotor fleft;
        private DcMotor fright;
        private DcMotor bright;
        private DcMotor bleft;


        @Override
        public void runOpMode() {
            int count = 0;
            String zone = "";

            bleft = hardwareMap.get(DcMotor.class, "bleft");
            bright = hardwareMap.get(DcMotor.class, "bright");
            fleft = hardwareMap.get(DcMotor.class, "fleft");
            fright = hardwareMap.get(DcMotor.class, "fright");
            distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

            bleft.setDirection(DcMotorSimple.Direction.REVERSE);
            fleft.setDirection(DcMotorSimple.Direction.REVERSE);


            waitForStart();

            //Starting Object Detection
            runMotorsTime(0.25, 1500);
            brakeMotors();
            turnLeft(0.25, 2100);

            while (opModeIsActive()) {
                // Measures Distance
                //Check Where Object Is
                if (distanceSensor.getDistance(DistanceUnit.INCH) > 25) {
                    turnRight(0.25, 50);

                    //Creating Zones
                    count = count + 1;
                    telemetry.addData("Turn_Num: ", count);

                    // Zone 1
                } else if (count > 5 && count < 16) {
                    zone = "Zone 1";
                    zone1();
                    startPark();
                    break;

                    // Zone 2
                } else if (count > 17 && count < 25) {
                    zone = "Zone 2";
                    zone2();
                    startPark();
                    break;

                    // Zone 3
                } else if (count > 27) {
                    zone = "Zone 3";
                    zone3();
                    startPark();
                    break;
                }
                telemetry.addData("", zone);
                telemetry.update();
            }
        }

        // Gets Pixel on Spike 1
        public void zone1() {
            runMotorsTime(0.25, 750);
            brakeMotors();
            runMotorsTime(-0.25, 250);
            turnRight(0.5, 350);
        }

        // Gets Pixel on Spike 2
        public void zone2() {
            runMotorsTime(0.25, 1300);
            brakeMotors();
        }

        // Gets Pixel on Spike 3
        public void zone3() {
            // turnRight(0.5, 250);
            runMotorsTime(0.25, 1200);
            brakeMotors();
        }

        public void startPark() {
            runMotorsTime(-0.5, 2000);
            strafeMotorsLeft(0.25, 1000);
            runMotorsTime(0.25, 100);
            brakeMotors();
            strafeMotorsRight(0.5, 5000);
        }

        public void brakeMotors() {
            bleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            bright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            fleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            fright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        public void runMotorsTime(double power, long motorTime) {
            bleft.setPower(power);
            bright.setPower(power * 0.95);
            fright.setPower(power * 0.95);
            fleft.setPower(power);

            sleep(motorTime);

            bleft.setPower(0);
            bright.setPower(0);
            fright.setPower(0);
            fleft.setPower(0);
        }

        public void strafeMotorsRight(double power, long motorTime) {
            bleft.setPower(-power);
            bright.setPower(power);
            fright.setPower(-power);
            fleft.setPower(power);

            sleep(motorTime);

            bleft.setPower(0);
            bright.setPower(0);
            fright.setPower(0);
            fleft.setPower(0);
        }

        public void strafeMotorsLeft(double power, long motorTime) {
            bleft.setPower(power);
            bright.setPower(-power);
            fright.setPower(power);
            fleft.setPower(-power);

            sleep(motorTime);

            bleft.setPower(0);
            bright.setPower(0);
            fright.setPower(0);
            fleft.setPower(0);
        }

        public void turnRight(double power, long motorTime) {
            bleft.setPower(power);
            bright.setPower(-power);
            fright.setPower(-power);
            fleft.setPower(power);

            sleep(motorTime);

            bleft.setPower(0);
            bright.setPower(0);
            fright.setPower(0);
            fleft.setPower(0);

        }


        public void turnLeft(double power, long motorTime) {
            bleft.setPower(-power);
            bright.setPower(power);
            fright.setPower(power);
            fleft.setPower(-power);

            sleep(motorTime);

            bleft.setPower(0);
            bright.setPower(0);
            fright.setPower(0);
            fleft.setPower(0);
        }
    }
}
