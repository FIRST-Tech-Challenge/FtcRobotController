package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

public class Vision extends SubsystemBase {
    Telemetry m_telemetry;
    WebcamName m_webcamName;
    VuforiaLocalizer m_vuforia;

    static final String vuforia_license = "ARKNcpL/////AAABmaul75WJu02hpEsBG/MnvsZ0aacsUMH0zc+d53A" +
            "KGDU3mzdXJQzSDPuea0rokovM0/U3INJNoaNvGx+Xnk9tFdgMVitg+hE32fMsH4f5KLF9CqJyqRynTBo55jfOsN4UbPMO6ij" +
            "MdpQg7PPUg8O7pd5HNOjoDx+MKgZ+FldA4uCCj5vEansKoP5++7V0a/E/0j4MClpdkUA/7cf9WSNS8opUnl9lAyNpOEiOa0b" +
            "q2KbzC5234XlaqzE7it5yl9QhstUyAfy1rRyRYc7ClclkuK1kleXepW2FQED5MsC3S+4buqtAe2pnJA7QyHJ3PGUBQd3L5PF" +
            "VVDeXRGHIF6ZKij3R6zKbWc6/NVSc2J7S5Uz2";

    public Vision(HardwareMap hardwareMap, Telemetry telemetry) {
        m_telemetry = telemetry;

        // Camera
        m_webcamName = hardwareMap.get(WebcamName.class, "camera");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = vuforia_license;
        parameters.cameraName = m_webcamName;
        m_vuforia = ClassFactory.getInstance().createVuforia(parameters);

        VuforiaTrackables ultimateGoal = m_vuforia.loadTrackablesFromAsset("UltimateGoal");

        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(ultimateGoal);

        m_telemetry.addLine("Vision Initialized");

    }

    @Override
    public void periodic() {
        m_telemetry.update();
    }
}
