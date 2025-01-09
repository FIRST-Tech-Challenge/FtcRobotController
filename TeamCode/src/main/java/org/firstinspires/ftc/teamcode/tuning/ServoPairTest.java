package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.configurations.ServoConf;

import java.util.HashMap;
import java.util.Map;
import java.util.List;
import java.util.ArrayList;
import java.util.ListIterator;

@Config
@TeleOp(name = "ServoPairTest")
public class ServoPairTest extends LinearOpMode {

    public enum Mode {
        MASTER_ONLY,
        SLAVE_ONLY,
        BOTH  // Very Dangerous : If slave and master have been moved through pairing rather than by using setPosition, once Both mode is activated, they will force to reach back to their latest programmed position, thus blocking each other and destroying the mecanism
    };

    public  static double   incrementStep  = 0.05;
    public  static boolean  useConfReverse = false;
    public  static boolean  masterReverse  = false;
    public  static boolean  slaveReverse   = false;
    public  static double   masterPos      = 0.0;
    public  static double   slavePos       = 0.0;
    public  static long     sleepMs        = 200;
    public  static String   masterName     = "";
    public  static String   slaveName      = "";
    public  static Mode     mode           = Mode.MASTER_ONLY;

    Servo   master;
    Servo   slave;

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        ServoConf masterConf = Configuration.s_Current.getServo(masterName);
        ServoConf slaveConf  = Configuration.s_Current.getServo(slaveName);

        master = hardwareMap.servo.get(masterConf.getName());
        slave  = hardwareMap.servo.get(slaveConf.getName());

        if(useConfReverse){
            if(masterConf.getReverse()) { master.setDirection(Servo.Direction.REVERSE); }
            else                        { master.setDirection(Servo.Direction.FORWARD); }
            if(slaveConf.getReverse())  { slave.setDirection(Servo.Direction.REVERSE);  }
            else                        { slave.setDirection(Servo.Direction.FORWARD);  }
        }
        else {
            if(masterReverse) { master.setDirection(Servo.Direction.REVERSE); }
            else              { master.setDirection(Servo.Direction.FORWARD); }
            if(slaveReverse)  { slave.setDirection(Servo.Direction.REVERSE);  }
            else              { slave.setDirection(Servo.Direction.FORWARD);  }
        }

        slavePos  = slave.getPosition();
        masterPos = master.getPosition();

        if(mode == Mode.MASTER_ONLY)     {
            mode = Mode.SLAVE_ONLY;
            master.getController().pwmDisable();
        }
        else if(mode == Mode.SLAVE_ONLY) {
            mode = Mode.BOTH;
        }
        else if(mode == Mode.BOTH)       {
            mode = Mode.MASTER_ONLY;
            slave.getController().pwmDisable();
        }

        waitForStart();

        // Scan servo till stop pressed.
        while(opModeIsActive()) {

            if(gamepad1.a) {
                if(mode == Mode.MASTER_ONLY)     {
                    mode = Mode.SLAVE_ONLY;
                    master.getController().pwmDisable();
                }
                else if(mode == Mode.SLAVE_ONLY) {
                    mode = Mode.MASTER_ONLY;
                    slave.getController().pwmDisable();
                }
                // We removed the capability to reach both mode, see explanation in the Mode enum declaration
            }

            // Adjust servo position with Left/Right bumpers
            if (gamepad1.left_bumper) {
                if(mode == Mode.MASTER_ONLY)      { masterPos = Math.max(0.00, masterPos - incrementStep); }
                else if(mode == Mode.SLAVE_ONLY)  { slavePos = Math.max(0.00, slavePos - incrementStep);   }
                else if(mode == Mode.BOTH)        {
                    slavePos  = Math.max(0.00, slavePos - incrementStep);
                    masterPos = Math.max(0.00, masterPos - incrementStep);
                }
            } else if (gamepad1.right_bumper) {
                if(mode == Mode.MASTER_ONLY)      { masterPos = Math.min(1.00, masterPos + incrementStep); }
                else if(mode == Mode.SLAVE_ONLY)  { slavePos = Math.min(1.00, slavePos + incrementStep);   }
                else if(mode == Mode.BOTH)        {
                    slavePos  = Math.min(1.00, slavePos + incrementStep);
                    masterPos = Math.min(1.00, masterPos + incrementStep);
                }
            }

            // Set the current servo to the target position
            if(mode == Mode.MASTER_ONLY)      { master.setPosition(masterPos); }
            else if(mode == Mode.SLAVE_ONLY)  { slave.setPosition(slavePos);   }
            else if(mode == Mode.BOTH)        {
                master.setPosition(masterPos);
                slave.setPosition(slavePos);
            }

            // Display telemetry
            telemetry.addData("1-Mode", mode);
            telemetry.addData("2-Master", masterName);
            telemetry.addData("2-Master Position", masterPos);
            telemetry.addData("3-Slave", slaveName);
            telemetry.addData("3-Slave Position", slavePos);
            telemetry.update();

            sleep(sleepMs);

        }

    }
}