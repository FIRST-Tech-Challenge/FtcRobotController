package org.firstinspires.ftc.teamcode.team10515;

import org.firstinspires.ftc.teamcode.team10515.control.EnhancedGamepad;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.System.currentTimeMillis;
import static org.firstinspires.ftc.teamcode.team10515.Robot.getEnhancedGamepad1;
import static org.firstinspires.ftc.teamcode.team10515.Robot.setEnhancedGamepad1;

@TeleOp(name = "Wobble Goal/Servo Test", group = "Test")

public class ForkliftTest extends OpMode {
//    public double servoPos = 0;

    public double forkliftPower = 0;

    public long currentTime = 0;
    public long lastTimeA = 0;
    public long lastTimeY = 0;

    public ElapsedTime btnPressedA;
    public ElapsedTime btnPressedY;

    public boolean toggle = true;

    UGMapTest robot = new UGMapTest();

    static final double COUNTS_PER_MOTOR_REV = 134.4;
    static final double DRIVE_GEAR_REDUCTION = 2;
    static final double WHEEL_DIAMETER_INCHES = 10.25 * 2; // Wobble Goal Mover Height
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV*DRIVE_GEAR_REDUCTION)/(WHEEL_DIAMETER_INCHES*Math.PI);

    @Override
    public void start(){
        telemetry.addData("Started", "Ready for Command");
        telemetry.update();
    }

    @Override
    public void init() {
        /* Initialize the hardware map*/
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Init", "Hello Ultimate Goal Robot");    //
        updateTelemetry(telemetry);

        setEnhancedGamepad1(new EnhancedGamepad(gamepad1));
        btnPressedA = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        btnPressedY = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    @Override
    public void loop() {
        //Trial 1
        /*
        if (getEnhancedGamepad1().isA() && btnPressedA.milliseconds() > 250) {
            telemetry.addLine("a pressed");
            servoPos += 0.01;
            btnPressedA.reset();
        } else if (getEnhancedGamepad1().isY() && btnPressedY.milliseconds() > 250) {
            telemetry.addLine("y pressed");
            servoPos -= 0.01;
            btnPressedY.reset();
        } else if (getEnhancedGamepad1().isB()) {
            servoPos = 0;
        } else if (getEnhancedGamepad1().isX()) {
            servoPos = 0.5;
        }

        if(getEnhancedGamepad1().isA() && btnPressedA.milliseconds() > 250){
            telemetry.addLine("a pressed");
            toggle = !toggle;
            btnPressedA.reset();
            if (toggle){
                servoPos = 0;
            }
            else {
                servoPos = 0.5;
            }
        }
         */

//        int target = robot.forkliftMotor.getCurrentPosition() + (int) (0.5 * WHEEL_DIAMETER_INCHES * Math.PI);
        int target = robot.forkliftMotor.getCurrentPosition() + (int)(0.75*COUNTS_PER_MOTOR_REV);
        telemetry.addLine("Running to: " + target);

        robot.forkliftMotor.setTargetPosition(target);

        robot.forkliftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.forkliftMotor.setPower(0.4);

        while(robot.forkliftMotor.isBusy() && !(getEnhancedGamepad1().isA())){
            telemetry.addLine("Running");
        }

        robot.forkliftMotor.setPower(0);

        while(!(getEnhancedGamepad1().isA())){
            telemetry.addLine("Waiting for user input (press A to run again)");
        }

//        robot.forkliftMotor.setPower(forkliftPower);

        telemetry.addLine("Forklift Power: " + forkliftPower);
        telemetry.addLine("Current Position: " + robot.forkliftMotor.getCurrentPosition());

    }
}


