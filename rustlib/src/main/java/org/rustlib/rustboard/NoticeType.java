package org.rustlib.rustboard;

public enum NoticeType {
    POSITIVE("positive"),
    NEGATIVE("negative"),
    NEUTRAL("neutral");
    static final String NOTICE_MESSAGE_KEY = "notice_message";
    static final String NOTICE_TYPE_KEY = "notice_type";
    static final String NOTICE_DURATION_KEY = "notice_duration";
    String value;

    NoticeType(String value) {
        this.value = value;
    }
}