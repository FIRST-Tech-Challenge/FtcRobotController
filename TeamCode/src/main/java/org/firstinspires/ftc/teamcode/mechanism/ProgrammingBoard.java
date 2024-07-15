package org.firstinspires.ftc.teamcode.mechanism;



import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class ProgrammingBoard {
    private static DcMotor leftFrontMotor_0;
    private static DcMotor leftBackMotor_1;
    private static DcMotor rightBackMotor_2;
    private static DcMotor rightFrontMotor_3;
    private static DcMotor intakeMotor;
    private static DcMotor armMotor;
    private static DcMotor testMotor;

    private static Servo clawServo;
    private static Servo wristServo;
    private static Servo droneServo;

    // Defines the motors.

    private double tickPerRotation;
    private boolean Board1Active;

    public void init(HardwareMap hwMap) {
        // Function runs during init phase of robot.

        leftFrontMotor_0 = hwMap.get(DcMotor.class, "leftFrontMotor_0");
        leftBackMotor_1 = hwMap.get(DcMotor.class, "leftBackMotor_1");
        rightBackMotor_2 = hwMap.get(DcMotor.class, "rightBackMotor_2");
        rightFrontMotor_3 = hwMap.get(DcMotor.class, "rightFrontMotor_3");
        testMotor = hwMap.get(DcMotor.class, "testMotor");
        // Maps motors.

        leftFrontMotor_0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor_1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor_3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        testMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Sets the mode. *See official FTC guide for more options.

        leftFrontMotor_0.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor_1.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor_2.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor_3.setDirection(DcMotor.Direction.REVERSE);
        // Sets the direction. You might have to change this if hardware team screws up. `-`

        intakeMotor = hwMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        armMotor = hwMap.get(DcMotor.class, "armMotor");
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        clawServo = hwMap.get(Servo.class, "clawServo");
        wristServo = hwMap.get(Servo.class, "wristServo");
        droneServo = hwMap.get(Servo.class, "droneServo");
        clawServo.setDirection(Servo.Direction.REVERSE);

        tickPerRotation = testMotor.getMotorType().getTicksPerRev();
        Board1Active = true;
    }

    public void setDCMotorPower(double leftFrontPower, double leftBackPower, double rightFrontPower, double rightBackPower){
        leftFrontMotor_0.setPower(leftFrontPower);
        leftBackMotor_1.setPower(leftBackPower);
        rightBackMotor_2.setPower(rightBackPower);
        rightFrontMotor_3.setPower(rightFrontPower);
        // Method allows for other classes to set the speed of each motor.
        // If hardware team mixes up the cord (as always), comment out the above commands to see which one is which.
    }

    public void setIntakePower(double intakePower){
        intakeMotor.setPower(-intakePower);
    }

    public void setArmPower(double armPower) {armMotor.setPower(armPower);}

    public void setClawServo (double clawAngle) {clawServo.setPosition(clawAngle);}

    public void setWristServo (double wristAngle) {wristServo.setPosition(wristAngle);}

    public void setDroneServo (double droneAngle) {droneServo.setPosition(droneAngle);}

    public void setTestMotor (double motorSpeed) {testMotor.setPower(motorSpeed);}

    public double getMotorRotation() {return testMotor.getCurrentPosition()/tickPerRotation;}

    public boolean isBoard1Active(){
        return Board1Active;
    }






}
