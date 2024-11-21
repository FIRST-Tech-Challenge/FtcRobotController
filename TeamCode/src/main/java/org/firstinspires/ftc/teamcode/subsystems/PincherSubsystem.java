package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;
import com.arcrobotics.ftclib.hardware.ServoEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.FTCDashboardPackets;
import org.firstinspires.ftc.teamcode.util.MatchRecorder.MatchLogger;

public class PincherSubsystem extends SubsystemBase {

    ServoEx finger1, finger2;
    private final FTCDashboardPackets dbp = new FTCDashboardPackets("FingerSubsystem");

    public enum FingerPositions {
        ZERO(0, AngleUnit.DEGREES),
        OPEN(45/2f, AngleUnit.DEGREES),
        CLOSED(45, AngleUnit.DEGREES);

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

    public PincherSubsystem(final ServoEx finger1, final ServoEx finger2) {
        this.finger1 = finger1;
        this.finger2 = finger2;
        finger1.setRange(0, 90, AngleUnit.DEGREES);
        finger2.setRange(0, 90, AngleUnit.DEGREES);
        finger1.setInverted(true); //  Might need to change it to finger2
        // MIGHT cause errors
        locomoteFinger(FingerPositions.ZERO);
    }

    double lastPower = Double.MIN_VALUE;

    public void locomoteFinger(FingerPositions position) {
        finger1.turnToAngle(position.getAngle(), position.getAngleUnit());
        finger2.turnToAngle(position.getAngle(), position.getAngleUnit());
        MatchLogger.getInstance().genericLog("Finger", MatchLogger.FileType.FINGER, position.name());
    }

}
