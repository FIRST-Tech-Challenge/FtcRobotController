/*7078 mecanumm drive code */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class UGRobot {

    private Telemetry telemetry;
    private LinearOpMode opMode;


    enum MoveDirection {FORWARD, BACKWARD, LEFT, RIGHT}

    private DcMotor rightLaunchMotor = null;
    private DcMotor leftLaunchMotor = null;
    private DcMotor pickup = null;
    private Servo launchServo;

    public void init(HardwareMap hardwareMap, Telemetry telemetryIn, LinearOpMode opModeIn) {

        telemetry = telemetryIn;
        opMode = opModeIn;

    }
}
