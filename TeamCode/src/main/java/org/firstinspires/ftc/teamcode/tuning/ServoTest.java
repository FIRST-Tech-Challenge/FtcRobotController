package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.configurations.Configuration;

import java.util.HashMap;
import java.util.Map;
import java.util.List;
import java.util.ArrayList;
import java.util.ListIterator;
import java.util.Collections;
import java.util.Comparator;

@Config
@TeleOp(name = "ServoTest")
public class ServoTest extends LinearOpMode {

    public static double   incrementStep  = 0.01;
    public static double   targetPos      = 0.0;
    public static long     sleepMs        = 200;
    public static boolean  reverseForce   = false;
    public static boolean  reverseFromConf = true;

    private static boolean holdPosition   = false;

    private final Map<String,Servo>                 servos = new HashMap<>();
    List<Map.Entry<String, Servo>>                  entryList;
    public ListIterator<Map.Entry<String, Servo>>   iterator;
    Map.Entry<String,Servo>                         current;

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        Configuration.s_Current.getForTuning().forEach((key, value) -> {
            Map.Entry<String, Boolean> hw = value.getHw(0);
            Servo temp = null;
            if(hw != null) { temp = hardwareMap.tryGet(Servo.class,hw.getKey());}
            if( reverseFromConf) {
                if (temp != null && value.getHw(0).getValue()) {
                    temp.setDirection(Servo.Direction.REVERSE);
                }
                else if (temp != null) {
                    temp.setDirection(Servo.Direction.FORWARD);
                }
            }
            else { temp.setDirection(Servo.Direction.FORWARD); }
            if (temp != null && reverseForce) { temp.setDirection(Servo.Direction.REVERSE); }

            if(temp != null) { servos.put(key,temp); }
        });

        if (!servos.isEmpty()) {

            entryList = new ArrayList<>(servos.entrySet());

            iterator = entryList.listIterator();
            if (iterator.hasNext()) {
                current = iterator.next();
                targetPos = current.getValue().getPosition();
            }

        }
        telemetry.addLine("Found " + servos.size() + " servos to tune");

        waitForStart();

        // Scan servo till stop pressed.
        while(opModeIsActive() && !servos.isEmpty()) {
            // Toggle caching position mode
            if (gamepad1.a) {
                holdPosition = !holdPosition;
            }

            // Cycle through servos with X/Y buttons
            if (gamepad1.x) {
                if (!holdPosition) {
                    // Disable PWM for the current servo if hold position is disabled
                    current.getValue().getController().pwmDisable();
                }
                if (iterator.hasNext()) {
                    current = iterator.next();
                } else {
                    iterator = entryList.listIterator();
                }
                targetPos = current.getValue().getPosition();
                current.getValue().getController().pwmEnable(); // Enable PWM for the new servo
            } else if (gamepad1.y) {
                if (!holdPosition) {
                    // Disable PWM for the current servo if hold position is disabled
                    current.getValue().getController().pwmDisable();
                }
                if (iterator.hasPrevious()) {
                    current = iterator.previous();
                } else {
                    iterator = entryList.listIterator(entryList.size());
                }// Enable PWM for the new servo
                targetPos = current.getValue().getPosition();
                current.getValue().getController().pwmEnable();
            }

            // Adjust servo position with Left/Right bumpers
            if (gamepad1.left_bumper) {
                targetPos = Math.max(0.00, targetPos - incrementStep); // Decrease position but don't go below 0
            } else if (gamepad1.right_bumper) {
                targetPos = Math.min(1.00, targetPos + incrementStep); // Increase position but don't exceed 1
            }

            // Set the current servo to the target position
            current.getValue().setPosition(targetPos);

            // Display telemetry
            telemetry.addData("Servo", current.getKey());
            telemetry.addData("Position", targetPos);
            telemetry.addData("Holding position", holdPosition);
            telemetry.addData("Reverse", current.getValue().getDirection());
            telemetry.update();

            sleep(sleepMs);

        }

    }
}