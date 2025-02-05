package org.firstinspires.ftc.teamcode.constants;


import static org.firstinspires.ftc.teamcode.constants.Constants.FileConstants.*;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utility.Debugger;
import org.opencv.core.Scalar;

import java.io.*;
import java.lang.reflect.*;

public final class ConstantsSaver {
    private final boolean debug;
    private final Debugger debugger;

    /**
     * Creates a new ConstantsSaver class with debugging disabled
     */
    public ConstantsSaver() {
        debug    = false;
        debugger = new Debugger(null);
    }

    /**
     * Creates a new ConstantsSaver class with debugging enabled.
     * @param telemetry The telemetry to display debug information
     */
    public ConstantsSaver(@NonNull Telemetry telemetry) {
        debug    = true;
        debugger = new Debugger(telemetry);
    }

    /**
     * <p>
     *     Attempts to save all of the values located in {@link Constants} to their corresponding
     *     text files located at:
     * </p>
     * <p>
     *     /sdcard/FIRST/java/src/org/firstinspires/ftc/team26396/Constants
     * </p>
     * <p>
     *     If their is no corresponding text file, one will be created.
     * </p>
     * <p>
     *     By default all errors are silently ignored. To enable debug mode, pass the OpModes
     *     telemetry object into the constructor of the ConstantsLoader class.
     * </p>
     */
    public void save() {
        for (Class<?> clazz : Constants.class.getDeclaredClasses()) {
            if (!Modifier.isStatic(clazz.getModifiers())) {
                String issue = "Skipped Class " + clazz.getSimpleName()
                        + "\nReason: Class Is Not Static";
                debugger.addMessage(issue);
                continue;
            }
            saveClass(clazz);
        }

        if (debug) debugger.displayAll();
    }

    private void saveClass(@NonNull Class<?> clazz) {
        String fileName = SD_CARD_PATH + clazz.getSimpleName() + ".txt";

        try (FileWriter fileWriter = new FileWriter(new File(fileName))) {
            Field[] fields = clazz.getFields();

            if (fields.length == 0) {
                String issue = "Skipped Class " + clazz.getSimpleName()
                        + "\nReason: No Accessible Fields";
                debugger.addMessage(issue);
                return;
            }

            for (Field field : fields) { saveField(fileWriter, field); }

            fileWriter.flush();
        } catch (IOException ioException) {
            String issue = "Failed To Create File " + fileName
                    + "\nReason: " + ioException.getMessage();
            debugger.addMessage(issue);
        }
    }

    private void saveField(@NonNull FileWriter fileWriter, @NonNull Field field) {
        if (!isSavable(field)) return;

        String line;
        String fieldName = field.getName();

        try {
            switch (SupportedType.fieldToType(field)) {
                case SPARK_FUN_POSE_2D:
                    line = fieldName + "=" + pose2DString((SparkFunOTOS.Pose2D) field.get(null));
                    break;
                case SCALAR:
                    line = fieldName + "=" + scalarString((Scalar) field.get(null));
                    break;
                case UNSUPPORTED:
                    String issue = "Failed To Save Field " + fieldName
                            + "\nReason: " + field.getType().getSimpleName() + " Is Not A"
                            + "Supported Type.";
                    debugger.addMessage(issue);
                    return;
                default: // All other types don't need anything special to save them
                    line = fieldName + "=" + field.get(null);
                    break;
            }
        } catch (IllegalAccessException | IllegalArgumentException ignored) { return; }

        try {
            fileWriter.write(line + "\n");
        } catch (IOException ioException) {
            String issue = "Failed To Save Field " + field.getName() + "\n"
                    + "Reason: " + ioException.getMessage();
            debugger.addMessage(issue);
        }
    }

    @NonNull private String pose2DString(@NonNull SparkFunOTOS.Pose2D pose2D) {
        return "Pose2D(" + pose2D.x + "," + pose2D.y + "," + pose2D.h + ")";
    }

    @NonNull private String scalarString(@NonNull Scalar scalar) {
        return "Scalar(" + scalar.val[0] + "," + scalar.val[1] + "," + scalar.val[3] + ")";
    }

    private boolean isSavable(@NonNull Field field) {
        int modifiers = field.getModifiers();

        boolean isPublic = Modifier.isPublic(modifiers);
        boolean isFinal  = Modifier.isFinal(modifiers);
        boolean isStatic = Modifier.isStatic(modifiers);

        if (!isPublic || isFinal || !isStatic) {
            String issue = "Failed To Save Field" + field.getName();

            if (!isPublic) issue += "Reason: Field Is Not Public";
            if (!isFinal)  issue += "Reason: Field Is Final";
            if (!isStatic) issue += "Reason: Field Is Not Static";

            debugger.addMessage(issue);
            return false;
        }
        return true;
    }
}