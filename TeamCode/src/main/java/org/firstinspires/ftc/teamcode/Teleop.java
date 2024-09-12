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
    private DcMotor arm = null;
    private Servo claw = null;
    private Servo horizontalWrist = null;
    private Servo verticalWrist = null;
    
    
    public void runOpMode(){
        //these are functions(go down)
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        arm = hardwareMap.get(DcMotor.class, "arm");
        claw = hardwareMap.get(Servo.class, "claw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");



        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        arm.setDirection(DcMotor.Direction.REVERSE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("Arm Position: ", arm.getCurrentPosition());
            telemetry.update();
            double vertical = 0.4 * gamepad1.left_stick_y;
            double strafe = -0.4 * gamepad1.left_stick_x;
            double turn = -0.35 * gamepad1.right_stick_x;
            double position1= 100;
            double position2 = 100;
            claw(position1, position2);
            robotMovement(vertical, strafe, turn);
            if((gamepad1.left_stick_button) || (gamepad1.right_stick_button)){
                brake();
            }
            
            
            double manualArmPower = gamepad1.right_trigger - gamepad1.left_trigger;
            double manualArmDeadband = 0.3;
            boolean manualMode = false;
            armMovement(manualArmPower, manualArmDeadband, manualMode);
        }
    }

    private void claw(double position1, double position2) {
        leftClaw.setPosition(position1);
        rightClaw.setPosition(position2);
    }
        //sets all motor power to 0
        public void brake(){
            frontLeft.setPower(0.0);
            frontRight.setPower(0.0);
            backLeft.setPower(0.0);
            backRight.setPower(0.0);
        }
        
        //this is the robot movement code we made earlier
        //the variables are defined in "while(opModeIsActive())"
        public void robotMovement(double vertical, double strafe, double turn){
            frontLeft.setPower(-vertical-strafe-turn);
            frontRight.setPower(-vertical+strafe+turn);
            backLeft.setPower(vertical+strafe-turn);
            backRight.setPower(vertical-strafe+turn); 
        }
        
        //This is the arm movement code we made earlier
        //the variables are defined in "while(opModeIsActive())"
        public void armMovement(double manualArmPower, double manualArmDeadband, boolean manualMode) {
            if (Math.abs(manualArmPower) > manualArmDeadband) {
                arm.setPower(manualArmPower);
                if (!manualMode) {
                    arm.setTargetPosition(arm.getCurrentPosition());
                    arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    arm.setPower(0.0);
                    manualMode = true;
                } else {
                    arm.setTargetPosition(arm.getCurrentPosition());
                    arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    arm.setPower(0.3);
                    manualMode = false;
                }
            } else if (Math.abs(manualArmPower) < manualArmDeadband) {
                arm.setPower(0.1);
            }
        }
}
