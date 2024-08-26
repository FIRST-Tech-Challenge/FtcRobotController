package com.acmerobotics.dashboard;

import static org.junit.jupiter.api.Assertions.assertTrue;

import com.acmerobotics.dashboard.config.reflection.ReflectionConfig;
import com.acmerobotics.dashboard.config.variable.BasicVariable;
import com.acmerobotics.dashboard.config.variable.ConfigVariable;
import com.acmerobotics.dashboard.config.variable.CustomVariable;
import java.util.Map;
import org.junit.jupiter.api.Test;

public class SerializationTests {

    public static boolean varEquals(ConfigVariable<?> v, ConfigVariable<?> v2) {
        if (v == null) {
            return v2 == null;
        }

        if (v2 == null) {
            return false;
        }

        if (v.getType() != v2.getType()) {
            return false;
        }

        if (v instanceof BasicVariable) {
            BasicVariable<?> bv = (BasicVariable<?>) v;
            BasicVariable<?> bv2 = (BasicVariable<?>) v2;

            if (bv.getValue() == null) {
                return bv2.getValue() == null;
            }

            return bv.getValue().equals(bv2.getValue());
        } else if (v instanceof CustomVariable) {
            CustomVariable bv = (CustomVariable) v;
            CustomVariable bv2 = (CustomVariable) v2;

            if (bv.getValue() == null) {
                return bv2.getValue() == null;
            }

            if (bv2.getValue() == null) {
                return false;
            }

            for (Map.Entry<String, ConfigVariable> e : bv.entrySet()) {
                if (!varEquals(e.getValue(), bv2.getVariable(e.getKey()))) {
                    return false;
                }
            }

            for (Map.Entry<String, ConfigVariable> e : bv2.entrySet()) {
                if (!varEquals(e.getValue(), bv.getVariable(e.getKey()))) {
                    return false;
                }
            }

            return true;
        }

        return false;
    }

    public static void assertSerDeIdentity(CustomVariable cv) {
        String s = DashboardCore.GSON.toJson(cv);
        System.out.println(s);
        ConfigVariable<?> cv2 = DashboardCore.GSON.fromJson(s, CustomVariable.class);
        System.out.println(DashboardCore.GSON.toJson(cv2));
        assertTrue(varEquals(cv, cv2), "Message: " + s);
    }

    public static class Pair {
        public String first;
        public Pair second;
    }

    public static class NullVariables {
        public static Boolean a = null;
        public static Integer b = null;
        public static Double c = null;
        public static String d = null;
        public static RobotStatus.OpModeStatus e = null;
        public static RobotStatus f = null;
        public static Pair g = new Pair();
    }

    @Test
    void addition() {
        assertSerDeIdentity(ReflectionConfig.createVariableFromClass(NullVariables.class));
    }

}

