package org.firstinspires.ftc.teamcode.Subsystems.Vision.DetectMarker;

import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.Config.MainConfig;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;

public class DetectMarkerThread implements Runnable{

    Robot robot;
    private final AllianceColor allianceColor = MainConfig.getAllianceColor();
    MarkerLocation markerLocation = MarkerLocation.NOT_FOUND;

    public DetectMarkerThread(Robot robot) {
        this.robot = robot;
    }

    /**
     * This method waits until the search for the marker is done, and then it return the marker
     * location. It waits until {@link DetectMarker#getSearchStatus() } returns
     * {@link SearchStatus#FOUND}. The markerLocation is updated every 1 second
     *
     * @see DetectMarker#getMarkerLocation()
     */
    @Override
    public void run() {

        DetectMarker detectMarker = new DetectMarker(robot, allianceColor);
        while (detectMarker.getSearchStatus() != SearchStatus.FOUND) {
            markerLocation = detectMarker.getMarkerLocation();
            try {
                Thread.sleep(3000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
//        return markerLocation;
    }

    public MarkerLocation getMarkerLocation() {
        return this.markerLocation;
    }
}
