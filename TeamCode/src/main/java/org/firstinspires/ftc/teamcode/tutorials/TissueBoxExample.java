package org.firstinspires.ftc.teamcode.tutorials;

public class TissueBoxExample {
    int MAX_TISSUES = 200;
    int currentTissueCount;

    public void init() {
        MAX_TISSUES = 200;
        currentTissueCount = 200;
    }

    public String getTissue() {
        if (currentTissueCount <= 0) {
            return "no more tissues!";
        }
        decrementTissues();
        return "tissue";
    }

    public double getPercentFull() {
        return (double)currentTissueCount / (double)MAX_TISSUES;
    }

    private void decrementTissues() {
        currentTissueCount = currentTissueCount - 2;
    }
}
