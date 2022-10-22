package org.firstinspires.ftc.teamcode.robots.catbot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
<<<<<<< Updated upstream:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robots/catbot/TestOpMode.java
=======
import com.qualcomm.robotcore.hardware.HardwareMap;
>>>>>>> Stashed changes:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robots/idk/TestOpMode.java
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Iron Core OpMode", group="Challenge")
public class TestOpMode extends OpMode {
    //variable setup
    private DcMotor motorFrontRight = null;
    private DcMotor motorBackLeft = null;
    private DcMotor motorFrontLeft = null;
    private DcMotor motorBackRight = null;
<<<<<<< Updated upstream:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robots/catbot/TestOpMode.java
    private DcMotor elevator = null;
    private Servo claw = null;
    // regular drive
    private double powerLeft = 0;
    private double powerRight = 0;
    // motor power
    private double powerElevator = 0;
=======
    private DcMotor motorElevator = null;
    private Servo clawServo = null;
    // regular drive
    private double powerLeft = 0;
    private double powerRight = 0;
    // mecanum types
    private double powerFrontLeft = 0;
    private double powerFrontRight = 0;
    private double powerBackLeft = 0;
    private double powerBackRight = 0;
    // elevator and claw
    private double powerElevator = 0;
    private double powerClaw = 0;
>>>>>>> Stashed changes:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robots/idk/TestOpMode.java
    //number variables
    private static final float DEADZONE = .1f;
    private static final int MAXELEVHEIGHT = Integer.MAX_VALUE;
    private static final int MINELEVHEIGHT = 0;
    private int currElevHeight = 0;
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing " + this.getClass()+"...");
        telemetry.addData("Status", "Hold right_trigger to enable debug mode");
        telemetry.update();
        motorFrontLeft = this.hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorBackLeft = this.hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorFrontRight = this.hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorBackRight = this.hardwareMap.get(DcMotor.class, "motorBackRight");
<<<<<<< Updated upstream:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robots/catbot/TestOpMode.java
        elevator = this.hardwareMap.get(DcMotor.class, "elevator");
        claw = this.hardwareMap.get(Servo.class, "claw");
=======
        motorElevator = this.hardwareMap.get(DcMotor.class, "motorElevator");
        clawServo = this.hardwareMap.get(Servo.class, "motorElevator");
>>>>>>> Stashed changes:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robots/idk/TestOpMode.java
        this.motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        this.motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
    }
    @Override
    public void loop() {
        //tankDrive();
        mechanumDrive();
<<<<<<< Updated upstream:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robots/catbot/TestOpMode.java
        elevatorMove();
        clawMove();
=======
        clawDrive();
>>>>>>> Stashed changes:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robots/idk/TestOpMode.java
    }
    public void tankDrive()
    {
        powerRight = 0;
        powerLeft = 0;


        if(Math.abs(gamepad1.left_stick_y) > DEADZONE)
        {
            powerLeft = gamepad1.left_stick_y;
        }
        if(Math.abs(gamepad1.right_stick_y) > DEADZONE)
        {
            powerRight = gamepad1.right_stick_y;
        }
        motorFrontRight.setPower(powerRight);
        motorFrontLeft.setPower(powerLeft);
        motorBackRight.setPower(powerRight);
        motorBackLeft.setPower(powerLeft);
    }
    public void mechanumDrive()
    {
        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
        final double v1 = r * Math.cos(robotAngle) - rightX;
        final double v2 = r * Math.sin(robotAngle) + rightX;
        final double v3 = r * Math.sin(robotAngle) - rightX;
        final double v4 = r * Math.cos(robotAngle) + rightX;
        motorFrontLeft.setPower(v1);
        motorFrontRight.setPower(v4);
        motorBackLeft.setPower(v3);
        motorBackRight.setPower(v2);
    }
<<<<<<< Updated upstream:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robots/catbot/TestOpMode.java
    public void elevatorMove()
    {
        powerElevator = 0;
        if(gamepad1.right_trigger > DEADZONE)
        {
            powerElevator = gamepad1.right_trigger;
        }
        if(gamepad1.left_trigger > DEADZONE)
        {
            powerElevator = -gamepad1.left_trigger;
        }
        elevator.setPower(powerElevator);
        currElevHeight += powerElevator;
    }
    public void clawMove()
    {
        telemetry.addData("Claw servo position:", claw.getPosition());
        if(gamepad1.left_bumper)
            claw.setPosition(.5);
        if(gamepad1.right_bumper)
            claw.setPosition(0);
=======
    public void clawDrive()
    {
        powerClaw = 0;
        powerElevator = 0;
        double up = 0;
        double down = 0;
        if (gamepad1.right_trigger > DEADZONE)
            up = gamepad1.right_trigger;
        if (gamepad1.left_trigger > DEADZONE)
            down = gamepad1.left_trigger;
        powerElevator = (up - down);
        if(gamepad1.right_bumper)
            powerClaw = 1.0;
        if(gamepad1.left_bumper)
            powerClaw = 0.0;
        motorElevator.setPower(powerElevator);
        clawServo.setPosition(powerClaw);





>>>>>>> Stashed changes:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robots/idk/TestOpMode.java
    }
}
