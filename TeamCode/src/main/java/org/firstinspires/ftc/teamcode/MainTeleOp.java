package org.firstinspires.ftc.teamcode;


//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "MainTeleOp")
public class MainTeleOp extends LinearOpMode{
    Boolean homeFlag = false;
    public void ClawOpen(Servo claw)
    {
        if (gamepad1.a) {
            claw.setPosition(0.5);
        }
    }
    public void ClawClosed(Servo claw)
    {
        if (gamepad1.b) {
            claw.setPosition(0.5);
        }
    }
    public void WristMethod(Servo wrist)
    {
        if (gamepad1.a) {
            wrist.setPosition(0.5);
        }
    }
    public void ArmMotorRaise(DcMotor armMotor)
    {
        armMotor.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int armTicks = 600;
        armMotor.setTargetPosition(armTicks);    //Sets Target Tick Position
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.05);
        homeFlag = false;
    }
    public void ArmMotorHome(DcMotor armMotor)
    {
        armMotor.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int armTicks = 0;
        armMotor.setTargetPosition(armTicks);    //Sets Target Tick Position
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.05);
        homeFlag = true;

    }
    public void ArmMotorCustom(DcMotor armMotor, int degrees)
    {
        //Total ticks in a revolution for 117 RPM motor: 1425
        //Total ticks for armMotor drivetrain revolution: 7125
        //90 degree rotation for armMotor drivetrain revolution: 1781.25
        int currentPosit = armMotor.getCurrentPosition();
        double ticksPerDegree = 7125.0/360.0;
        double convertPosit = currentPosit/ticksPerDegree;
        if (convertPosit > degrees)
        {
            armMotor.setDirection(DcMotor.Direction.FORWARD);
        }
        else
        {
            armMotor.setDirection(DcMotor.Direction.REVERSE);
        }
        int convert = (int) (degrees*ticksPerDegree);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setTargetPosition(convert);    //Sets Target Tick Position
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.05);
    }
    public void ViperMotorMethod(DcMotor viperMotor)
    {
        viperMotor.setDirection(DcMotor.Direction.FORWARD);
        viperMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int viperTicks = 600;
        viperMotor.setTargetPosition(viperTicks);    //Sets Target Tick Position
        viperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperMotor.setPower(0.05);
    }
    @Override
    public void runOpMode() throws InterruptedException{
        // Initialization Code Goes Here
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        DcMotor armMotor;
        DcMotor viperMotor;
        Servo claw;
        Servo wrist;
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        viperMotor = hardwareMap.get(DcMotor.class, "viperMotor");
//        claw = hardwareMap.get(Servo.class, "claw");
//        wrist = hardwareMap.get(Servo.class, "wrist");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        while(opModeIsActive()){ //while loop for when program is active
            //Code repeated during teleop goes here
            //Analogous to loop() method in OpMode
            double drive;
            double turn;
            double strafe;
            double fLeftPow, fRightPow, bLeftPow, bRightPow;
            double speedMultiplier = 0.75;

            // Reverse the right side motors
            rightFront.setDirection(DcMotor.Direction.REVERSE);
            rightBack.setDirection(DcMotor.Direction.REVERSE);

            //drive inputs
            drive = gamepad1.left_stick_y * -1;
            turn = gamepad1.right_stick_x;
            strafe = gamepad1.left_stick_x;
            if (gamepad1.right_trigger > 0) {speedMultiplier = 1;}
            if (gamepad1.left_trigger > 0) {speedMultiplier = 0.25;}

            //drive calculations
            fLeftPow = Range.clip((drive + turn + strafe) * speedMultiplier, -1, 1);
            bLeftPow = Range.clip((drive + turn - strafe) * speedMultiplier, -1, 1);
            fRightPow = Range.clip((drive - turn - strafe) * speedMultiplier, -1, 1);
            bRightPow = Range.clip((drive - turn + strafe) * speedMultiplier, -1, 1);

            leftFront.setPower(fLeftPow);
            leftBack.setPower(bLeftPow);
            rightFront.setPower(fRightPow);
            rightBack.setPower(bRightPow);

            if (gamepad1.x) {ArmMotorRaise(armMotor);}
            if (gamepad1.y) {ArmMotorHome(armMotor);}
            if (gamepad1.a) {ArmMotorCustom(armMotor, 85);}

            if (armMotor.getCurrentPosition() < 15  )
            {
                if (homeFlag)
                {
                    armMotor.setPower(0);
                    armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                else {homeFlag = false;}
            }

            motorTelemetry(armMotor, "armMotor");
            motorTelemetry(viperMotor, "viperMotor");
            telemetry.update();
        }

    }
    private void motorTelemetry( DcMotor motor, String caption)
    {
        int position;
        position = motor.getCurrentPosition();
        telemetry.addData(caption, position);
    }
}