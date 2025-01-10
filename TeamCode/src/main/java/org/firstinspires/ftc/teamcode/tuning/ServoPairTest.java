package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.configurations.ConfServo;

import java.util.Iterator;
import java.util.Map;


@Config
@TeleOp(name = "ServoPairTest")
public class ServoPairTest extends LinearOpMode {

    public enum Mode {
        MASTER_ONLY,
        SLAVE_ONLY,
        BOTH  // Very Dangerous : If slave and master have been moved through pairing rather than by using setPosition, once Both mode is activated, they will force to reach back to their latest programmed position, thus blocking each other and destroying the mecanism
    };

    public  static double   incrementStep       = 0.05;
    public  static boolean  reverseFromConf     = true;
    public  static boolean  reverseForceMaster  = false;
    public  static boolean  reverseForceSlave   = false;
    public  static double   posMaster           = 0.0;
    public  static double   posSlave            = 0.0;
    public  static long     sleepMs             = 200;
    public  static String   nameMaster          = "";
    public  static String   nameSlave           = "";
    public  static Mode     mode                = Mode.MASTER_ONLY;

    Servo   master;
    Servo   slave;

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        ConfServo masterConf = Configuration.s_Current.getServoForTuning(nameMaster);
        ConfServo slaveConf = Configuration.s_Current.getServoForTuning(nameSlave);

        Map.Entry<String, Boolean> hwMaster = masterConf.getHw(0);
        Map.Entry<String, Boolean> hwSlave = slaveConf.getHw(0);

        telemetry.addLine(hwMaster.getKey());
        telemetry.addLine(hwSlave.getKey());

        master = null;
        slave = null;

        if(hwMaster != null) { master = hardwareMap.servo.get(hwMaster.getKey()); }
        if(hwSlave != null) { slave = hardwareMap.servo.get(hwSlave.getKey()); }

        if (reverseFromConf) {
            if (master != null && hwMaster.getValue()) {
                master.setDirection(Servo.Direction.REVERSE);
            }
            else if (master != null) {
                master.setDirection(Servo.Direction.FORWARD);
            }
            if (slave != null && hwSlave.getValue()) {
                slave.setDirection(Servo.Direction.REVERSE);
            }
            else if (slave != null) {
                slave.setDirection(Servo.Direction.FORWARD);
            }
        }
        else {
            if (master != null) {
                master.setDirection(Servo.Direction.FORWARD);
            }
            if (slave != null) {
                slave.setDirection(Servo.Direction.FORWARD);
            }
        }

        if (master != null && reverseForceMaster) {
            master.setDirection(Servo.Direction.REVERSE);
        }
        if (slave != null && reverseForceSlave) {
            slave.setDirection(Servo.Direction.REVERSE);
        }

        if(master != null && slave != null) {
            posSlave = slave.getPosition();
            posMaster = master.getPosition();

            if (mode == Mode.MASTER_ONLY) {
                mode = Mode.SLAVE_ONLY;
                master.getController().pwmDisable();
            } else if (mode == Mode.SLAVE_ONLY) {
                mode = Mode.BOTH;
            } else if (mode == Mode.BOTH) {
                mode = Mode.MASTER_ONLY;
                slave.getController().pwmDisable();
            }
        }
        telemetry.update();

        waitForStart();

        // Scan servo till stop pressed.
        while(opModeIsActive() && master != null && slave != null) {

            if(gamepad1.a) {
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
                // We removed the capability to reach both mode, see explanation in the Mode enum declaration
            }

            // Adjust servo position with Left/Right bumpers
            if (gamepad1.left_bumper) {
                if(mode == Mode.MASTER_ONLY)      { posMaster = Math.max(0.00, posMaster - incrementStep); }
                else if(mode == Mode.SLAVE_ONLY)  { posSlave = Math.max(0.00, posSlave - incrementStep);   }
                else if(mode == Mode.BOTH)        {
                    posSlave  = Math.max(0.00, posSlave - incrementStep);
                    posMaster = Math.max(0.00, posMaster - incrementStep);
                }
            } else if (gamepad1.right_bumper) {
                if(mode == Mode.MASTER_ONLY)      { posMaster = Math.min(1.00, posMaster + incrementStep); }
                else if(mode == Mode.SLAVE_ONLY)  { posSlave = Math.min(1.00, posSlave + incrementStep);   }
                else if(mode == Mode.BOTH)        {
                    posSlave  = Math.min(1.00, posSlave + incrementStep);
                    posMaster = Math.min(1.00, posMaster + incrementStep);
                }
            }

            // Set the current servo to the target position
            if(mode == Mode.MASTER_ONLY)      { master.setPosition(posMaster); }
            else if(mode == Mode.SLAVE_ONLY)  { slave.setPosition(posSlave);   }
            else if(mode == Mode.BOTH)        {
                master.setPosition(posMaster);
                slave.setPosition(posSlave);
            }

            // Display telemetry
            telemetry.addData("1-Mode", mode);
            telemetry.addData("2-Master", nameMaster);
            telemetry.addData("2-Master Position", posMaster);
            telemetry.addData("3-Slave", nameSlave);
            telemetry.addData("3-Slave Position", posSlave);
            telemetry.update();

            sleep(sleepMs);

        }

    }
}