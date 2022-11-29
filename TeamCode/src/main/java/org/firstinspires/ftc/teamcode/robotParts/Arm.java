package org.firstinspires.ftc.teamcode.robotParts;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm extends RobotPart {

    private enum ArmHeight {
        LOW(0),
        MID(0),
        HIGH(0),
        CUSTOM(0);

        private int position;
        public int getPosition() {
            return this.position;
        }
        public void setPosition(int position) {
            this.position = position;
        }
        ArmHeight(int position) {
            this.position = position;
        }
    }

    private int armTarget = 0;

    private enum ServoPosition {
        FRONT(0),
        MID(0),
        BACK(0),
        CUSTOM(0);

        private double position;
        public double getPosition() {
            return this.position;
        }
        public void setPosition(double position) {
            this.position = position;
        }
        ServoPosition(double position) {
            this.position = position;
        }
    }

    private double servoTarget = 0;

    public void init(HardwareMap map, Telemetry.Item telemetry) {
        telemetry = telemetry;

        // motors
        motors.put("armLeft", map.get(DcMotorEx.class, "armLeft"));
        motors.put("armRight", map.get(DcMotorEx.class, "armRight"));
        resetEncoders();

        // servus
        servos.put("rotate", map.get(Servo.class, "rotate"));

    }

    public void update() {

    }
}
