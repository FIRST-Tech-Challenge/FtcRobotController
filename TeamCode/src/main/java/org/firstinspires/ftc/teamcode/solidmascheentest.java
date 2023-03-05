package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;                //imports from FIRST
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class solidmascheentest extends OpMode {

    public enum LiftState {
        LIFT_START,
        LIFT_EXTEND,
        LIFT_DUMP,
        LIFT_RETRACT
    };
    LiftState liftState= LiftState.LIFT_START;

    private DcMotor frontLeft;
    private DcMotor frontRight;                                         //Declaring Motor varibles
    private DcMotor backLeft;
    private DcMotor backRight;



    private CRServo Left;
    private DcMotor Crain;
    private DcMotor Spin;

    private Rev2mDistanceSensor distance;

    ElapsedTime liftTimer = new ElapsedTime();

     final double DUMP_IDLE=0; // the idle position for the dump servo
     final double DUMP_DEPOSIT=-1; // the dumping position for the dump servo
     final double DUMP_TIME=2;

    final int LIFT_LOW=0; // the low encoder position for the lift
    final int LIFT_HIGH=-3000; // the high encoder position for the lift
    public void init()
    {
        liftTimer.reset();

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");                            //mapping motors from control hub
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");


        Left = hardwareMap.get(CRServo.class, "Lefts");
        Crain = hardwareMap.get(DcMotor.class, "Crane");
        Spin = hardwareMap.get(DcMotor.class, "Spin");

        distance = hardwareMap.get(Rev2mDistanceSensor.class, "distance");

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);                 //setting direction of drive train
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        Spin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Spin.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Crain.setTargetPosition(LIFT_LOW);
        Crain.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

        public void loop(){
        telemetry.addData("asdf",liftState);
        telemetry.update();
        Crain.setPower(1);
            double turn;
            double throttle;
            boolean strafeLeft;
            boolean strafeRight;

            float pickup;                                   //setting varibles from conteroler imputs
            float dropoff;
            boolean spinpowerup;
            boolean spinpowerdown;
            double crainpower;
            boolean spincenter;
            boolean opspincenter;
            boolean burst;

            throttle = gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;
            strafeLeft = gamepad1.left_bumper;
            strafeRight = gamepad1.right_bumper;



            crainpower = gamepad2.right_stick_y;
            spinpowerup = gamepad2.dpad_right;

            spinpowerdown =gamepad2.dpad_left;
            pickup = gamepad2.left_trigger;
            dropoff = gamepad2.right_trigger;


            switch (liftState) {
                case LIFT_START:
                    // Waiting for some input
                    if (gamepad1.x) {
                        // x is pressed, start extending
                        Crain.setTargetPosition(LIFT_HIGH);
                        liftState = LiftState.LIFT_EXTEND;
                    }
                    break;
                case LIFT_EXTEND:
                    // check if the lift has finished extending,
                    // otherwise do nothing.
                    if (Math.abs(Crain.getCurrentPosition() - LIFT_HIGH) < 10) {
                        // our threshold is within
                        // 10 encoder ticks of our target.
                        // this is pretty arbitrary, and would have to be
                        // tweaked for each robot.

                        // set the lift dump to dump
                        Left.setPower(DUMP_DEPOSIT);

                        liftTimer.reset();
                        liftState = LiftState.LIFT_DUMP;
                    }
                    break;
                case LIFT_DUMP:
                    if (liftTimer.seconds() >= DUMP_TIME) {
                        // The robot waited long enough, time to start
                        // retracting the lift
                       // Left.setTargetPosition(DUMP_IDLE);
                        Crain.setTargetPosition(LIFT_LOW);
                        liftState = LiftState.LIFT_RETRACT;
                    }
                    break;
                case LIFT_RETRACT:
                    if (Math.abs(Crain.getCurrentPosition() - LIFT_LOW) < 10) {
                        liftState = LiftState.LIFT_START;
                    }
                    break;
                default:
                    // should never be reached, as liftState should never be null
                    liftState = LiftState.LIFT_START;
            }




            if (strafeRight) {
                frontLeft.setPower(-.8);
                frontRight.setPower(1);                         //conecting motor varibles to controler inputs
                backLeft.setPower(1);
                backRight.setPower(-1);
            }
            if (strafeLeft) {
                frontLeft.setPower(.8);
                frontRight.setPower(-1);
                backLeft.setPower(-1);
                backRight.setPower(1);
            }


            frontLeft.setPower(throttle);
            frontRight.setPower(throttle);
            backLeft.setPower(throttle);
            backRight.setPower(throttle);

            frontLeft.setPower(-turn);
            frontRight.setPower(turn);
            backLeft.setPower(-turn);
            backRight.setPower(turn);

            Crain.setPower(crainpower);

            if (spinpowerup){
                Spin.setPower(1);
            }
            if (spinpowerdown){
                Spin.setPower(-1);
            }

            if (!spinpowerdown && !spinpowerup ){
                Spin.setPower(0);
            }
            if (pickup>0) {

                Left.setPower(-1);
            }

            if (dropoff>0){

                Left.setPower(1);

            }

            if (dropoff == 0 && pickup == 0){

                Left.setPower(0);

            }
            /*
            while (burst) {
                Left.setPower(.8);
                sleep(500);
                Left.setPower(0);
                sleep(1500);
                if (burst == false) {
                    break;
                }
            }
            */



            //telemetry.addData("distance",distance.getDistance(DistanceUnit.INCH));
            //telemetry.update();
        }


    }

