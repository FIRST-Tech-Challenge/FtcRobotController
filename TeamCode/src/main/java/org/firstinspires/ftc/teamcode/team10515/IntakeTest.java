package org.firstinspires.ftc.teamcode.team10515;

//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import org.firstinspires.ftc.teamcode.team10515.control.EnhancedGamepad;
//
//import static org.firstinspires.ftc.teamcode.team10515.Robot.getEnhancedGamepad1;
//import static org.firstinspires.ftc.teamcode.team10515.Robot.setEnhancedGamepad1;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.team10515.control.EnhancedGamepad;

import static java.lang.System.currentTimeMillis;
import static org.firstinspires.ftc.teamcode.team10515.Robot.getEnhancedGamepad1;
import static org.firstinspires.ftc.teamcode.team10515.Robot.setEnhancedGamepad1;

@TeleOp(name = "Intake Test", group = "Test")

public class IntakeTest extends OpMode {
    UGMapTest robot = new UGMapTest();
    public double intakeMotorSpeed = 0;
    public long currentTime = 0;
    public long lastTimeA = 0;
    public long lastTimeY = 0;

    public ElapsedTime btnPressedA;
    public ElapsedTime btnPressedY;

    @Override
    public void init() {
        robot.init(hardwareMap);
        telemetry.addData("Robot OpMode starting: ", "Intake Test");
        updateTelemetry(telemetry);
        setEnhancedGamepad1(new EnhancedGamepad(gamepad1));
        btnPressedA = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        btnPressedY = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    @Override
    public void loop() {
        if (getEnhancedGamepad1().isA() && btnPressedA.milliseconds() > 250) {
            telemetry.addLine("a pressed");
            intakeMotorSpeed += 0.1;
            btnPressedA.reset();
        } else if (getEnhancedGamepad1().isY() && btnPressedY.milliseconds() > 250) {
            telemetry.addLine("y pressed");
            intakeMotorSpeed -= 0.1;
            btnPressedY.reset();
            lastTimeY = currentTimeMillis();
        }
        robot.intakeMotor.setPower(intakeMotorSpeed);
        telemetry.addLine("Intake Motor Speed: " + intakeMotorSpeed);
    }
}
