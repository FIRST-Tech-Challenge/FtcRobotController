// mr_stoffer was here
// hello there
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@TeleOp(name="Gengar Driving Solo", group="Gengar")

public class GengarDwiveTwainSolo extends OpMode {
    
    private ElapsedTime runtime = new ElapsedTime();
    
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private DcMotor armMotor1 = null;
    private Servo intake = null;
    double intakePosition = 0.45;
    
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        armMotor1 = hardwareMap.get(DcMotor.class, "armmotor1");
        intake = hardwareMap.get(Servo.class, "Intake");
        
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        armMotor1.setDirection(DcMotor.Direction.REVERSE);

        intake.setDirection(Servo.Direction.FORWARD);
        
        
        telemetry.addData("Status", "Initialized");
    }
    @Override
    public void init_loop() {
    }
    
    @Override
    public void start() {
        runtime.reset();
    }
    
    @Override
    public void loop() {
        double leftFrontPower;
        double leftBackPower;
        double rightFrontPower;
        double rightBackPower;
        double armPower1;

        boolean sensitivity = gamepad1.right_bumper;
        //boolean motorSensitivity = gamepad1.left_bumper;
        
        double x = gamepad1.left_stick_x; // 1.5;
        double y = -gamepad1.left_stick_y; // 1.5;
        /*if (motorSensitivity == true) {
            x = gamepad1.left_stick_x;
            y = -gamepad1.left_stick_y;
        }
        */
        
        double rightX = gamepad1.right_stick_x;
        double r = Math.hypot(x, y);
        double robotAngle = Math.atan2(y, x) - Math.PI / 4;
        double leftTrigger = gamepad1.left_trigger;
        double rightTrigger = gamepad1.right_trigger;
        boolean leftBumper2 = gamepad1.dpad_down;
        boolean rightBumper2 = gamepad1.dpad_up;
        boolean armUp1 = gamepad1.b;
        
        if (sensitivity == true) {
            rightX = gamepad1.right_stick_x / 2;
        }
        
        if (gamepad1.dpad_down) {
            intakePosition = 0.3;
        }
        if (gamepad1.dpad_up) {
            intakePosition = 0.45;
        }
        if (armUp1) {
            armMotor1.setMode(DcMotor.RunMode.RESET_ENCODERS);
            armMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            armMotor1.setTargetPosition(-2100);
            armMotor1.setPower(0.5);
            armMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (armMotor1.isBusy()) {
            }
        }
        
        if(gamepad1.right_trigger > 0  && armMotor1.getCurrentPosition() > 2100) {

armMotor1.setPower(0.5);

}

//else if(gamepad1.left_trigger && nameOfMotor.getCurrentPosition() > minimum) {

//nameOfMotor.setPower(-0.5);

//}

//else {

//nameOfMotor.setPower(0);

//}
        
        leftFrontPower = (r * Math.cos(robotAngle) + rightX);
        rightFrontPower = (r * Math.sin(robotAngle) - rightX);
        leftBackPower = (r * Math.sin(robotAngle) + rightX);
        rightBackPower = (r * Math.cos(robotAngle) - rightX);
        armPower1 = rightTrigger - leftTrigger;
       
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
        armMotor1.setPower(armPower1*0.75);
        intake.setPosition(intakePosition);
        
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("LeftTrigger", leftTrigger);
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }
}
