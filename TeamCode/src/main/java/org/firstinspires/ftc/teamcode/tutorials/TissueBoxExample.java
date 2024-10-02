package org.firstinspires.ftc.teamcode.tutorials;

public class TissueBoxExample {
    int MAX_TISSUES = 100;
    int currentTissueCount;

    public void init() {
        MAX_TISSUES = 100;
        currentTissueCount = 100;
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
        currentTissueCount--;
        currentTissueCount = currentTissueCount - 3;
        currentTissueCount -= 3;
    }
}
