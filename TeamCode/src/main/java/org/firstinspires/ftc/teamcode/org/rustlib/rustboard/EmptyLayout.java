package org.firstinspires.ftc.teamcode.org.rustlib.rustboard;

public class EmptyLayout extends RustboardLayout {
    public EmptyLayout() {
        super(null);
    }

    @Override
    public void setNodeValue(String id, String value) {

    }

    @Override
    public void setNodeValue(String id, Object value) {

    }

    public double getDoubleValue(String id, double defaultValue) {
        return defaultValue;
    }

    public boolean getBooleanValue(String id) {
        return false;
    }

    public String getInputValue(String id) {
        return "";
    }

    public String getSelectedValue(String id) {
        return "";
    }
}
