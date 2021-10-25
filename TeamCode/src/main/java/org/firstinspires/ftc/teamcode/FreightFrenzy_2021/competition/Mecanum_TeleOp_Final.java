package org.firstinspires.ftc.teamcode.FreightFrenzy_2021.competition;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;

@TeleOp(name = "Mecanum TeleOp Final", group = "Linear OpMode")
public class Mecanum_TeleOp_Final extends LinearOpMode {

    private DcMotor LF = null;
    private DcMotor RF = null;
    private DcMotor LB = null;
    private DcMotor RB = null;
    private DcMotor Intake = null;
    private DcMotor Spin = null;
    private DcMotor Slide = null;
    private Servo Rotate = null;
    private Servo Push = null;
    private ArrayList<Double[]> speedList = new ArrayList<Double[]>();
    private ElapsedTime runtime = new ElapsedTime();

    double rotate = 0;
    double speed = 0.5;
    boolean reverse = false;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        LF  = hardwareMap.get(DcMotor.class, "LF");
        RF = hardwareMap.get(DcMotor.class, "RF");
        LB  = hardwareMap.get(DcMotor.class, "LB");
        RB = hardwareMap.get(DcMotor.class, "RB");
        Slide  = hardwareMap.get(DcMotor.class, "Slide");
        Intake  = hardwareMap.get(DcMotor.class, "Intake");
        Spin = hardwareMap.get(DcMotor.class, "Spin");

        LF.setDirection(DcMotor.Direction.REVERSE);
        RF.setDirection(DcMotor.Direction.FORWARD);
        LB.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.FORWARD);

        Intake.setDirection(DcMotor.Direction.REVERSE);
        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Slide.setDirection(DcMotor.Direction.FORWARD);
        Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Spin.setDirection(DcMotor.Direction.FORWARD);
        Spin.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Rotate = hardwareMap.get(Servo.class, "Rotate");
        Rotate.setDirection(Servo.Direction.FORWARD);
        Push = hardwareMap.get(Servo.class, "Push");


        double LFPower;
        double RFPower;
        double LBPower;
        double RBPower;

        double intakePower = 0;
        double spinPower = 0;
        int initialHeight = Slide.getCurrentPosition();

        waitForStart();

        boolean releasedRightBumper = true;
        boolean releasedLeftBumper = true;
        boolean releasedX = true;
        boolean releasedLB = true;
        boolean releasedRB = true;
        boolean releasedLT = true;
        boolean releasedRT = true;
        boolean releasedA = true;
        boolean releasedB = true;
        boolean releasedY = true;

        boolean releasedDD = true;
        boolean releasedDU = true;
        boolean releasedDL = true;
        boolean toggleX = true;
        boolean toggleRB = true;
        boolean toggleLB = true;
        boolean toggleRT = true;
        boolean toggleLT = true;

        while (opModeIsActive()) {
            runtime.reset();

            double drive = -gamepad1.left_stick_y;
            double strafe  = -gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            if(gamepad1.right_bumper) {
                if(releasedRightBumper && releasedLeftBumper) {
                    increaseSpeed(0.05);
                    releasedRightBumper = false;
                }
            } else if(!releasedRightBumper){
                releasedRightBumper = true;
            }

            if(gamepad1.left_bumper){
                if(releasedRightBumper && releasedLeftBumper) {
                    decreaseSpeed(0.05);
                    releasedLeftBumper = false;
                }
            } else if (!releasedLeftBumper){
                releasedLeftBumper = true;
            }

            if(gamepad1.a){
                if(releasedA) {
                    Slide.setTargetPosition(initialHeight);

                    Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Slide.setPower(0.6);
                }

            } else if(!releasedA){
                releasedA = true;
            }
            if(gamepad1.b){
                if(releasedB) {
                    Slide.setTargetPosition(initialHeight + 750);

                    Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Slide.setPower(0.6);
                }

            } else if(!releasedB){
                releasedB = true;
            }
            if(gamepad1.y){
                if(releasedY) {
                    Slide.setTargetPosition(initialHeight + 1400);
                    Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Slide.setPower(0.6);
                }
            } else if(!releasedY){
                releasedY = true;
            }


            if (gamepad1.x) {
                if (releasedX){
                    if (toggleX) {
                        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        telemetry.addLine("BREAK");
                        toggleX = false;
                    } else {
                        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        telemetry.addLine("FLOAT");
                        toggleX = true;
                    }
                    releasedX = false;
                }
            } else if (!releasedX){
                releasedX = true;
            }

//            if (gamepad1.dpad_up) {
//                if (releasedDU){
//                    Slide.setPower(0.6);
//                    releasedDU = false;
//                }
//            } else if (!releasedDU){
//                releasedDU = true;
//            }
//            if (gamepad1.dpad_down) {
//                if (releasedDD){
//                    Slide.setPower(-0.6);
//                    releasedDD = false;
//                }
//            } else if (!releasedDD){
//                releasedDD = true;
//            }
//            if (gamepad1.dpad_left) {
//                if (releasedDL){
//                    Slide.setPower(0.0);
//                    releasedDL = false;
//                }
//            } else if (!releasedDL){
//                releasedDL = true;
//            }


            if (gamepad1.left_bumper) {
                if (releasedLB){
                    if (toggleLB) {
                        intakePower = 0.8;
                        telemetry.addLine("INTAKE STARTS");
                        toggleLB = false;
                    } else {
                        intakePower = 0;
                        telemetry.addLine("INTAKE STOPS");
                        toggleLB = true;
                    }
                    releasedLB = false;
                }
            } else if (!releasedLB){
                releasedLB = true;
            }

            if (gamepad2.right_bumper) {
                if (releasedRB){
                    if (toggleRB) {
                        spinPower = 0.5;
                        telemetry.addLine("SPIN STARTS");
                        toggleRB = false;
                    } else {
                        spinPower = 0;
                        telemetry.addLine("SPIN STOPS");
                        toggleRB = true;
                    }
                    releasedRB = false;
                }
            } else if (!releasedRB){
                releasedRB = true;
            }

            if(gamepad2.right_trigger == 1 && Rotate.getPosition() > 0.4){
                if (releasedRT){
                    Push.setPosition(0);
                    releasedRT = false;
                }
            } else if (!releasedRT){
                Push.setPosition(0.4);
                releasedRT = true;
            }

            if(gamepad2.left_trigger == 1){
                if (releasedLT){
                    if (toggleLT) {
                        Rotate.setPosition(0.03);
                        telemetry.addLine("ROTATE STARTS");
                        toggleLT = false;
                    } else {
                        Rotate.setPosition(1.0);
                        telemetry.addLine("ROTATE STOPS");
                        toggleLT = true;
                    }
                    releasedLT = false;
                }
            } else if (!releasedLT){
                releasedLT = true;
            }


            LFPower  = Range.clip(gamepad1.left_trigger + speed*(drive + rotate - strafe), -1.0, 1.0) ;
            LBPower  = Range.clip(gamepad1.left_trigger + speed*(drive + rotate + strafe), -1.0, 1.0) ;
            RFPower  = Range.clip(gamepad1.right_trigger + speed*(drive - rotate + strafe), -1.0, 1.0) ;
            RBPower  = Range.clip(gamepad1.right_trigger + speed*(drive - rotate - strafe), -1.0, 1.0) ;


            Double currentSpeed[] = {LFPower, LBPower, RFPower, RBPower};
            speedList.add(currentSpeed);

            LF.setPower(LFPower);
            RF.setPower(RFPower);
            LB.setPower(LBPower);
            RB.setPower(RBPower);
            Intake.setPower(intakePower);
            Spin.setPower(spinPower);

            telemetry.addData("Servo","Rotate (%.2f), Push (%.2f)", Rotate.getPosition(), Push.getPosition());
            telemetry.addLine("Intake: " + intakePower);
            telemetry.addLine("Spin: " + spinPower);
            telemetry.addLine("Slide Current: " + Slide.getCurrentPosition());
            telemetry.addLine("Slide Target: " + Slide.getTargetPosition());
            telemetry.addData("Front Motors", "LF (%.2f), RF (%.2f)", LFPower, RFPower);
            telemetry.addData("Back Motors", "LB (%.2f), RB (%.2f)", LBPower, RBPower);
            telemetry.addData("Controller", "X (%.2f), Y (%.2f)", strafe, drive);
            telemetry.addData("Speed:", speed);

            /*if(loop == 200){
                if (reverse){
                    reverse = false;
                }
                telemetry.addData("duration of a loop: %.2f", runtime.milliseconds());
                sleep(100);
                loop = 0;
            }*/
            telemetry.update();
            //loop++;

//            released = true;
        }
    }

    private void decreaseSpeed(double s) {
        double decreased = speed - s;
        if (decreased < 0) {
            speed = 0;
            return;
        }
        speed = decreased;
    }

    private void increaseSpeed(double s) {
        double increased = speed + s;
        if (1 < increased) {
            speed = 1;
            return;
        }
        speed = increased;
    }
}