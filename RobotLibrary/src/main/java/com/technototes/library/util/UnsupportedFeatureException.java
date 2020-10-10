package com.technototes.library.util;

public class UnsupportedFeatureException extends Exception {
    public UnsupportedFeatureException() {
        throw new RuntimeException("This feature is not yet supported, and may never be");
    }

    public UnsupportedFeatureException(String unsupportedFeature) {
        throw new RuntimeException(unsupportedFeature + " is not supported yet, and may never be");
    }

    public UnsupportedFeatureException(String unsupportedFeature, String reason) {
        throw new RuntimeException(unsupportedFeature + " is not supported yet because " + reason);
    }
}
