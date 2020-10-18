package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/*
This class defines a ringObject, an object defining a ring or set of rings detected by TensorFlow.

A ringObject has a few attributes: a type and 4 positions as given by the detecting class.
 */

public class ringObject {
    // Static methods
    private float round(float number_to_round, int places) {
        number_to_round *= Math.pow(10,places);
        float temp = (float)(int)number_to_round;
        temp /= Math.pow(10,places);
        return temp;
    }
    // Properties
    String label = "";
    float[] positions = new float[4];
    // Constructors
    ringObject(String newlabel, float left, float top, float right, float bottom) {
        label = newlabel;
        positions[0] = round(left, 3);
        positions[1] = round(top, 3);
        positions[2] = round(right, 3);
        positions[3] = round(bottom, 3);
    }
    ringObject() {
        label = "";
        Arrays.fill(positions, 0.0f);
    }
    // Accessors
    String getLabel() {
        return label;
    }

    float[] getPositions() {
        return positions;
    }

    float getPositions(int a) {
        return positions[a];
    }

    float getLeft() {
        return positions[0];
    }

    float getTop() {
        return positions[1];
    }

    float getRight() {
        return positions[2];
    }

    float getBottom() {
        return positions[3];
    }
    // Mutators
    void setLabel(String newlabel) {
        label = newlabel;
    }
    // Important methods

    /* detectRings: Given an already-initialized TFObjectDetector tfod, this detects rings and returns
    an ArrayList with all of the different ring objects. Based off the example
     */
    public static ArrayList<ringObject> detectRings(TFObjectDetector tfod) {
        ArrayList<ringObject> objects = new ArrayList<>();
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                int size = updatedRecognitions.size();
                for (Recognition recognition : updatedRecognitions) {
                    ringObject object = new ringObject(recognition.getLabel(), recognition.getLeft(), recognition.getTop(), recognition.getRight(), recognition.getBottom());
                    objects.add(object);
                }
            }
        }
        return objects;
    }

}
