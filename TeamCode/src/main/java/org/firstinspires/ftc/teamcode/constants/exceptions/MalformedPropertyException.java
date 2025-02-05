package org.firstinspires.ftc.teamcode.constants.exceptions;

import androidx.annotation.NonNull;

public final class MalformedPropertyException extends Exception {
    public String name;
    public String reason;
    public String value;

    public MalformedPropertyException(
            @NonNull String name,
            @NonNull String reason,
            @NonNull String value
    ) {
        this.name = name;
        this.reason = reason;
        this.value = value;
    }
}