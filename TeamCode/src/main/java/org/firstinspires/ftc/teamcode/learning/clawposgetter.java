package org.firstinspires.ftc.teamcode.learning;

import android.graphics.Color;

import androidx.annotation.ColorInt;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.COMPETITIONCODE.data.servoManger;
@Disabled
@TeleOp(name="CLAW GET")
public class clawposgetter extends LinearOpMode {
    private servoManger clawServo = new servoManger();
    private servoManger clawRotateServo = new servoManger();
    private servoManger clawRotateServo2 = new servoManger();
    @Override
    public void runOpMode() throws InterruptedException {
        clawServo.init(hardwareMap, "cs");
        clawRotateServo.init(hardwareMap, "crs");
        clawRotateServo2.init(hardwareMap, "crs2");
        RevColorSensorV3 color;
        color = hardwareMap.get(RevColorSensorV3.class, "color");
        color.enableLed(true);
        waitForStart();
        while(!isStopRequested()){
            double pos = clawServo.getServoPosition();
            Color currentColor = new Color();
            telemetry.addLine("Blue:" + String.valueOf(color.blue()));
            telemetry.addLine("Green:" + String.valueOf(color.green()));
            telemetry.addLine("Red:" + String.valueOf(color.red()));
            double pos2 = clawRotateServo.getServoPosition();
            clawRotateServo2.setServoPosition(0.5);
            String posString = Double.toString(pos);
            String pos2String = Double.toString(pos2);
            if(gamepad2.b) {
                clawRotateServo.setServoPosition(0.1);
            }
            if(gamepad2.a) {
                clawRotateServo.setServoPosition(0.0);
            }
            if(gamepad2.y) {
                clawRotateServo.setServoPosition(0.7);
            }
            if(gamepad2.right_bumper){
                clawServo.setServoPosition(0.39);
            }
            if(gamepad2.left_bumper){
                clawServo.setServoPosition(0.0);
            }
            telemetry.addLine("clawServo pos:" + posString);
            telemetry.addLine("clawServoRotate pos:" + pos2String);
            telemetry.update();
        }
    }

}
