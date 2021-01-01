package org.firstinspires.ftc.teamcode.team10515;

import org.firstinspires.ftc.teamcode.team10515.control.EnhancedGamepad;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.System.currentTimeMillis;
import static org.firstinspires.ftc.teamcode.team10515.Robot.getEnhancedGamepad1;
import static org.firstinspires.ftc.teamcode.team10515.Robot.setEnhancedGamepad1;

@TeleOp(name = "Wobble Goal/Servo Test", group = "Test")

public class ForkliftTest extends OpMode {
    public double servoPos = 0;

    public long currentTime = 0;
    public long lastTimeA = 0;
    public long lastTimeY = 0;

    public ElapsedTime btnPressedA;
    public ElapsedTime btnPressedY;

    public boolean toggle = true;

    UGMapTest robot = new UGMapTest();

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

        robot.forkliftServo.setPosition(servoPos);

        telemetry.addLine("Servo Position: " + servoPos);

    }
}


