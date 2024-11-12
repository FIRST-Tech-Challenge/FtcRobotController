package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.arcrobotics.ftclib.hardware.ServoEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.FTCDashboardPackets;
import org.firstinspires.ftc.teamcode.util.MatchRecorder.MatchLogger;

public class FingerSubsystem extends SubsystemBase {

    ServoEx finger1, finger2;
    private final FTCDashboardPackets dbp = new FTCDashboardPackets("FingerSubsystem");

    public enum FingerPositions {
        ZERO(0, AngleUnit.DEGREES),
        OPEN(45, AngleUnit.DEGREES),
        CLOSED(90, AngleUnit.DEGREES);

        private final float angle;
        private final AngleUnit angleUnit;

        FingerPositions(float angle, AngleUnit angleUnit) {
            this.angle = angle;
            this.angleUnit = angleUnit;
        }

        public float getAngle() {
            return angle;
        }

        public AngleUnit getAngleUnit() {
            return angleUnit;
        }
    }

    public FingerSubsystem(final Servo servo) {
        
    }

    public FingerSubsystem(final ServoEx finger1, final ServoEx finger2) {
        this.finger1 = finger1;
        this.finger2 = finger2;
        finger1.setRange(0, 90, AngleUnit.DEGREES);
        finger2.setRange(0, 90, AngleUnit.DEGREES);
        finger1.setInverted(true); //  Might need to change it to finger2
        // MIGHT cause errors
        locomoteFinger(FingerPositions.ZERO);
    }

    double lastPower = Double.MIN_VALUE;

    /*private void locomoteFinger(double forward, double backward) {
        double power = forward-backward;
        power++;
        power/=2d;
        if (power != lastPower) {
            MatchLogger.getInstance().genericLog("Finger", MatchLogger.FileType.FINGER, "Power Value", power);
        }
        lastPower = power;
        finger.setPosition(power);

        dbp.createNewTelePacket();
        dbp.info("Finger Power: "+power+", "+finger.getPosition()+", "+forward+", "+backward);
        dbp.send(false);
    }*/

    public void locomoteFinger(FingerPositions position) {
        finger1.turnToAngle(position.getAngle(), position.getAngleUnit());
        finger2.turnToAngle(position.getAngle(), position.getAngleUnit());
        MatchLogger.getInstance().genericLog("Finger", MatchLogger.FileType.FINGER, position.name());
    }

}
