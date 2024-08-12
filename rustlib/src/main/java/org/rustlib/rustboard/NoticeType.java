package org.rustlib.rustboard;

public enum NoticeType {
    POSITIVE("positive"),
    NEGATIVE("negative"),
    NEUTRAL("neutral");

    String value;

    NoticeType(String value) {
        this.value = value;
    }
}