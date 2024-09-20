//to find is control f
//This is the basic teleop code we can use for every robot with little modification
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Into The Deep Teleop")

public class Teleop extends LinearOpMode {
    //The wheels
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;

    //The two joints on the arm
    private DcMotor jointOne = null;
    private DcMotor jointTwo = null;

    //The claw
    private Servo claw = null;

    //the two servos for the wrist
    private Servo horizontalWrist = null;
    private Servo verticalWrist = null;

    private ColorSensor colorSensor = null;
    private DistanceSensor distanceSensor = null;


    public void runOpMode() {
        //control hub
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");//gamepad1 //conf 0
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");//gamepad1 //conf 1
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");//gamepad1  //conf 2
        backRight = hardwareMap.get(DcMotor.class, "backRight");//gamepad1 //conf 3

        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");//conf 0
        distanceSensor = hardwareMap.get(DistanceSensor.class,"distanceSensoe");//conf 1

        //expansion hub
        jointOne = hardwareMap.get(DcMotor.class, "jointOne");//gamepad2 //conf 1
        jointTwo = hardwareMap.get(DcMotor.class, "jointTwo");//gamepad2 //conf 2

        claw = hardwareMap.get(Servo.class, "claw");//gamepad2 //conf 0
        horizontalWrist = hardwareMap.get(Servo.class, "horizontalWrist");//gamepad2 //conf 1
        verticalWrist = hardwareMap.get(Servo.class, "verticalWrist");//gamepad2 //conf 2


        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        //TODO change the joints directions
        jointOne.setDirection(DcMotor.Direction.REVERSE);
        jointTwo.setDirection(DcMotor.Direction.REVERSE);

        jointOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        jointOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("First joints Position: ", jointOne.getCurrentPosition());
            telemetry.addData("Second joints Position: ", jointTwo.getCurrentPosition());
            telemetry.update();

            double vertical = 0.4 * gamepad1.left_stick_y;
            double strafe = -0.4 * gamepad1.left_stick_x;
            double turn = -0.35 * gamepad1.right_stick_x;
            robotMovement(vertical, strafe, turn);
            //gamepad1
//            if (((gamepad1.left_stick_x != 0) || (gamepad1.left_stick_y != 0)) || ((gamepad1.right_stick_x != 0) || (gamepad1.right_stick_y != 0))) {
//                //Makes robot faster while x button is pressed
//                if(gamepad1.x){
//                    double vertical = 0.6 * gamepad1.left_stick_y;
//                    double strafe = -0.6 * gamepad1.left_stick_x;
//                    double turn = -0.55 * gamepad1.right_stick_x;
//                //Makes robot slower while y button is pressed
//                }else if(gamepad1.y){
//                    double vertical = 0.2 * gamepad1.left_stick_y;
//                    double strafe = -0.2 * gamepad1.left_stick_x;
//                    double turn = -0.15 * gamepad1.right_stick_x;
//                //Base speed when nothing is pressed
//                }else {
//
//                }
//            }
            if ((gamepad2.left_stick_button) || (gamepad2.right_stick_button)) {
                brake();
            }
            double manualArmDeadband = 0.3;//Makes sure you don't move the arm with every small movement, makes it so you have to push it a little bit for it to move
            double jointOnePower = 0.6;
            double jointTwoPower = 0.6;
            newArm(manualArmDeadband, jointOnePower, jointTwoPower);


            //The second gamepad controls the arm
//            if((gamepad2.left_trigger != 0) || (gamepad2.right_trigger != 0)) {
//                double manualArmPower = gamepad2.right_trigger - gamepad2.left_trigger;
//                double manualArmDeadband = 0.3;
//               boolean manualMode = false;
//                armMovement(manualArmPower, manualArmDeadband, manualMode);
//            }


            //Opening and closing for horizontal wrist
            int Hpos1 = 100;
            int Hpos2 = 0;
            //opening and closing for vertical wrist
            int Vpos1 = 100;
            int Vpos2 = 0;
            wrist(Hpos1, Hpos2, Vpos1, Vpos2);
            int open = 100;
            int close = 0;
            claw(open,close);



        }
    }

//    private void claw(double position1) {
//        claw.setPosition(position1);
//    }

    //gamepad1
    //moves robot chassis
    public void robotMovement(double vertical, double strafe, double turn) {
        frontLeft.setPower(-vertical - strafe - turn);
        frontRight.setPower(-vertical + strafe + turn);
        backLeft.setPower(vertical + strafe - turn);
        backRight.setPower(vertical - strafe + turn);
    }

    //gamepad1
    //emergency brakes
    public void brake() {
        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backLeft.setPower(0.0);
        backRight.setPower(0.0);
    }

    //gamepad2
    //controls wrist
    private void wrist(int Hpos1, int Hpos2, int Vpos1, int Vpos2) {
        //For the wrist to move up and down
        //moves wrist to the right
        if (gamepad2.dpad_right) {
            horizontalWrist.setPosition(Hpos1);
        //moves wrist to the left
        } else if (gamepad2.dpad_left) {
            horizontalWrist.setPosition(Hpos2);
        }

        //For the wrist to move up and down
        //Moves wrist up
        if (gamepad2.dpad_up) {
            verticalWrist.setPosition(Vpos1);
        //Moves wrist down
        } else if (gamepad2.dpad_down) {
            verticalWrist.setPosition(Vpos2);
        }
    }


    private void claw(int open, int close) {
        if (gamepad2.a) {
            claw.setPosition(open);
        }else if (gamepad2.b) {
            claw.setPosition(close);
        }
    }

    //Gamepad2
    //For a double jointed arm that works with the right joystick
    public void newArm(double manualArmDeadband, double jointOnePower, double jointTwoPower) {
        //when the right stick is pushed up and down, it moves the bottom arm joint
        if (Math.abs(gamepad2.right_stick_y) > manualArmDeadband) {
            jointOne.setPower(jointOnePower * gamepad2.right_stick_y);
        //When the right stick is pushed left to right, it moves  the second arm joint
        } else if (Math.abs(gamepad2.right_stick_x) > manualArmDeadband) {
            jointTwo.setPower(jointTwoPower * gamepad2.right_stick_x);
        }else{
            jointOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            jointOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            jointOne.setPower(0.0);
            jointTwo.setPower(0.0);
        }
    }


    //This is the arm movement code we made earlier
    //the variables are defined in "while(opModeIsActive())"
//        public void armMovement(double manualArmPower, double manualArmDeadband, boolean manualMode) {
//            if (Math.abs(manualArmPower) > manualArmDeadband) {
//                arm.setPower(manualArmPower);
//                if (!manualMode) {
//                    arm.setTargetPosition(arm.getCurrentPosition());
//                    arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    arm.setPower(0.0);
//                    manualMode = true;
//                } else {
//                    arm.setTargetPosition(arm.getCurrentPosition());
//                    arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    arm.setPower(0.3);
//                    manualMode = false;
//                }
//            } else if (Math.abs(manualArmPower) < manualArmDeadband) {
//                arm.setPower(0.1);
//            }
//        }


}
