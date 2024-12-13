package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;
import com.arcrobotics.ftclib.hardware.ServoEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.FTCDashboardPackets;
import org.firstinspires.ftc.teamcode.util.MatchRecorder.MatchLogger;

import java.util.Objects;

public class PincherSubsystem extends SubsystemBase {

    ServoEx finger1, finger2;
    private final FTCDashboardPackets dbp = new FTCDashboardPackets("FingerSubsystem");

    public static float MAX_ANGLE = 5 / 2f;

    public enum FingerPositions {
        ZERO(0, AngleUnit.DEGREES),
        OPEN(MAX_ANGLE/2f, AngleUnit.DEGREES),
        CLOSED(MAX_ANGLE, AngleUnit.DEGREES);

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

    public FingerPositions currentFingerPosition;

    public PincherSubsystem(final ServoEx finger1, final ServoEx finger2) {
        this.finger1 = finger1;
        this.finger2 = finger2;
        Objects.requireNonNull(finger1);
        Objects.requireNonNull(finger2);
        finger1.setRange(0, MAX_ANGLE, AngleUnit.DEGREES);
        finger2.setRange(0, MAX_ANGLE, AngleUnit.DEGREES);

        //finger1.setInverted(true); //  Might need to change it to finger2
        //finger1.setInverted(true); //  Might need to change it to finger1
        // MIGHT cause errors
        //locomoteFinger(FingerPositions.ZERO);
    }

    double lastPower = Double.MIN_VALUE;

    public void locomoteFinger(FingerPositions position) {
        //finger1.turnToAngle(position.getAngle(), position.getAngleUnit());
        //finger2.turnToAngle(position.getAngle(), position.getAngleUnit());

        double angleScale = position.getAngle() / MAX_ANGLE;

        //finger1.setPosition(1f-angleScale);
        //finger2.setPosition(angleScale);
        finger1.turnToAngle((MAX_ANGLE-position.getAngle())/3f, position.getAngleUnit());
        finger2.turnToAngle((position.getAngle())/3f, position.getAngleUnit());
        //dbp.info("ANGLE: "+angleScale);
        //dbp.info("POSITION: "+finger1.getPosition()+ ", "+finger2.getPosition());
        //dbp.info("OBJECT: "+finger1+ ", "+finger2);
        String debug = "Angle: %f\nTargetPos: %f\nPosition: %f\nPositionName: %s";
        debug = String.format(debug, position.getAngle(), angleScale, finger1.getPosition(), position.name());
        dbp.info(debug);
        dbp.send(true);

        currentFingerPosition = position;
        MatchLogger.getInstance().genericLog("Finger", MatchLogger.FileType.FINGER, position.name());
    }

    public void closeFinger() {
        locomoteFinger(FingerPositions.CLOSED);
    }

    public void openFinger() {
        locomoteFinger(FingerPositions.OPEN);
    }

    public void zeroFinger() {
        locomoteFinger(FingerPositions.ZERO);
    }

    public boolean isFingerReady() {
        if (currentFingerPosition == null) {
            return false;
        }
        double angleScale = currentFingerPosition.getAngle() / MAX_ANGLE;
        return ((finger1.getAngle() == 1f-angleScale) && (finger2.getAngle() == angleScale));
    }

}
