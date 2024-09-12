//to find is control f
//This is the basic teleop code we can use for every robot with little modification
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Ali's TeleOp")

public class Teleop extends LinearOpMode {
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    //Thr two joints on the arm
    private DcMotor jointOne = null;
    private DcMotor jointTwo = null;
    private Servo claw = null;
    //the two servos for the wrist
    private Servo horizontalWrist = null;
    private Servo verticalWrist = null;


    public void runOpMode() {
        //control hub
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        //expansion hub
        jointOne = hardwareMap.get(DcMotor.class, "jointOne");
        jointTwo = hardwareMap.get(DcMotor.class, "jointTwo");
        claw = hardwareMap.get(Servo.class, "claw");
        horizontalWrist = hardwareMap.get(Servo.class, "horizontalWrist");
        verticalWrist = hardwareMap.get(Servo.class, "verticalWrist");


        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        //TODO change the joints directions
        jointOne.setDirection(DcMotor.Direction.REVERSE);
        jointTwo.setDirection(DcMotor.Direction.REVERSE);
        jointOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        jointOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("First joints Position: ", jointOne.getCurrentPosition());
            telemetry.addData("Second joints Position: ", jointTwo.getCurrentPosition());
            telemetry.update();
            //The first gamepad controls the movement
            if (((gamepad1.left_stick_x != 0) || (gamepad1.left_stick_y != 0)) || ((gamepad1.right_stick_x != 0) || (gamepad1.right_stick_y != 0))) {
                double vertical = 0.4 * gamepad1.left_stick_y;
                double strafe = -0.4 * gamepad1.left_stick_x;
                double turn = -0.35 * gamepad1.right_stick_x;
                robotMovement(vertical, strafe, turn);
            }
            if ((gamepad2.left_stick_button) || (gamepad2.right_stick_button)) {
                brake();
            }

            double manualArmDeadband = 0.3;
            double jointOnePower = 0.3;
            double jointTwoPower = 0.3;
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

    //this is the robot movement code we made earlier
    //the variables are defined in "while(opModeIsActive())"
    public void robotMovement(double vertical, double strafe, double turn) {
        frontLeft.setPower(-vertical - strafe - turn);
        frontRight.setPower(-vertical + strafe + turn);
        backLeft.setPower(vertical + strafe - turn);
        backRight.setPower(vertical - strafe + turn);
    }
    public void brake() {
        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backLeft.setPower(0.0);
        backRight.setPower(0.0);
    }

    private void wrist(int Hpos1, int Hpos2, int Vpos1, int Vpos2) {
        if (gamepad2.dpad_up) {
            horizontalWrist.setPosition(Hpos1);
        } else if (gamepad2.dpad_down) {
            horizontalWrist.setPosition(Hpos2);
        }


        if (gamepad2.dpad_right) {
            horizontalWrist.setPosition(Vpos1);
        } else if (gamepad2.dpad_left) {
            horizontalWrist.setPosition(Vpos2);
        }
    }

    private void claw(int open, int close) {
        if (gamepad2.a) {
            claw.setPosition(open);
        }else if (gamepad2.b) {
            claw.setPosition(close);
        }
    }
    //sets all motor power to 0

    public void newArm(double manualArmDeadband, double jointOnePower, double jointTwoPower) {
        //when the right stick is pushed, it moves the robots first join back and forth
        if (Math.abs(gamepad2.right_stick_y) > manualArmDeadband) {
            jointOne.setPower(jointOnePower * gamepad2.right_stick_y);
        } else if (Math.abs(gamepad2.right_stick_x) > manualArmDeadband) {
            jointTwo.setPower(jointTwoPower * gamepad2.left_stick_x);
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
