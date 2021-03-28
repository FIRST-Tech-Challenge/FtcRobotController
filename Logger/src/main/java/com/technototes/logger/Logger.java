package com.technototes.logger;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.technototes.logger.entry.BooleanEntry;
import com.technototes.logger.entry.Entry;
import com.technototes.logger.entry.NumberBarEntry;
import com.technototes.logger.entry.NumberEntry;
import com.technototes.logger.entry.NumberSliderEntry;
import com.technototes.logger.entry.StringEntry;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.annotation.Annotation;
import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

/**
 * The class to manage logging
 *
 * @author Alex Stedman
 */
public class Logger {

    private Entry<?>[] runEntries, initEntries;
    private Set<Entry<?>> unindexedRunEntries, unindexedInitEntries;
    private Telemetry telemetry;
    private OpMode opMode;
    /**
     * The divider between the tag and the entry for telemetry (default ':')
     */
    public char captionDivider = ':';

    /**
     * Instantiate the logger
     *
     * @param op The OpMode class
     */
    public Logger(OpMode op) {
        opMode = op;
        telemetry = op.telemetry;
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        unindexedRunEntries = new LinkedHashSet<>();
        unindexedInitEntries = new LinkedHashSet<>();
        configure(op);
        runEntries = generate(unindexedRunEntries);
        initEntries = generate(unindexedInitEntries);

    }

    private void configure(Object root) {
        for (Field field : root.getClass().getDeclaredFields()) {
            try {
                Object o = field.get(root);
                if (isFieldAllowed(field)) {
                    if (o instanceof Loggable) {
                        configure(o);
                    } else if (field.isAnnotationPresent(Log.class) || field.isAnnotationPresent(Log.Number.class) ||
                            field.isAnnotationPresent(Log.NumberSlider.class) || field.isAnnotationPresent(Log.NumberBar.class)
                            || field.isAnnotationPresent(Log.Boolean.class)) {
                        if (field.getType().isPrimitive() || o instanceof String) {
                            set(field.getDeclaredAnnotations(), field, root);
                        } else if (getCustom(o) != null) {
                            set(field.getDeclaredAnnotations(), getCustom(o));
                        } else if(o instanceof Stated) {
                            set(field.getDeclaredAnnotations(), ((Stated) o)::getState);
                        }
                    }
                }
            } catch (IllegalAccessException ignored) {

            }
        }
        for (Method m : root.getClass().getDeclaredMethods()) {
            set(m.getDeclaredAnnotations(), m, root);
        }
    }

    //TODO make list and do sort with comparators
    //List<List<Entry<?>>
    private Entry<?>[] generate(Set<Entry<?>> a) {
        Entry<?>[] returnEntry = new Entry[20];
        List<Entry<?>> unindexed = new ArrayList<>();
        for (Entry<?> e : a) {
            int index = e.getIndex();
            if (index != -1) {
                Entry<?> other = returnEntry[index];
                if (other == null) {
                    returnEntry[index] = e;
                } else {
                    if (e.getPriority() > other.getPriority()) {
                        unindexed.add(other);
                        returnEntry[index] = e;
                    } else {
                        unindexed.add(e);
                    }
                }
            } else {
                unindexed.add(e);
            }
        }
        for (int i = 0; unindexed.size() > 0; i++) {
            if (returnEntry[i] == null) {
                returnEntry[i] = unindexed.remove(0);
            }
        }

        return returnEntry;

    }


    private void update(Entry<?>[] choice) {
        for (int i = 0; i < choice.length; i++) {
            telemetry.addLine((i > 9 ? i + "| " : i + " | ") + (choice[i] == null ? "" :
                    choice[i].getTag().replace('`', captionDivider) + choice[i].toString()));
        }
        telemetry.update();
    }

    /**
     * Update the logged run items in temeletry
     */
    public void runUpdate() {
        update(runEntries);
    }

    /**
     * Update the logged init items in temeletry
     */
    public void initUpdate() {
        update(initEntries);
    }

    private void set(Annotation[] a, Method m, Object root) {
        set(a, () -> {
            try {
                return m.invoke(root);
            } catch (IllegalAccessException | InvocationTargetException e) {
                e.printStackTrace();
            }
            return null;
        });
    }

    private void set(Annotation[] a, Field m, Object root) {
        set(a, () -> {
            try {
                return m.get(root);
            } catch (IllegalAccessException e) {
                e.printStackTrace();
            }
            return null;
        });
    }
    @SuppressWarnings({"unchecked"})
    private void set(Annotation[] a, Supplier<?> m) {
        boolean init = false, run = true;
        Entry<?> e = null;
        for (Annotation as : a) {
            if (as instanceof Log.NumberSlider) {
                e = new NumberSliderEntry(((Log.NumberSlider) as).name(), (Supplier<Number>) m,
                        ((Log.NumberSlider) as).index(), ((Log.NumberSlider) as).min(),
                        ((Log.NumberSlider) as).max(), ((Log.NumberSlider) as).scale(),
                        ((Log.NumberSlider) as).color(), ((Log.NumberSlider) as).sliderBackground(),
                        ((Log.NumberSlider) as).outline(), ((Log.NumberSlider) as).slider());
                e.setPriority(((Log.NumberSlider) as).priority());
                break;
            } else if (as instanceof Log.NumberBar) {
                e = new NumberBarEntry(((Log.NumberBar) as).name(), (Supplier<Number>) m,
                        ((Log.NumberBar) as).index(), ((Log.NumberBar) as).min(),
                        ((Log.NumberBar) as).max(), ((Log.NumberBar) as).scale(),
                        ((Log.NumberBar) as).color(), ((Log.NumberBar) as).completeBarColor(),
                        ((Log.NumberBar) as).outline(), ((Log.NumberBar) as).incompleteBarColor());
                e.setPriority(((Log.NumberBar) as).priority());
                break;
            } else if (as instanceof Log.Number) {
                e = new NumberEntry(((Log.Number) as).name(), (Supplier<Number>) m,
                        ((Log.Number) as).index(), ((Log.Number) as).color(),
                        ((Log.Number) as).numberColor());
                e.setPriority(((Log.Number) as).priority());
                break;
            } else if (as instanceof Log) {
                e = new StringEntry(((Log) as).name(), (Supplier<String>) m,
                        ((Log) as).index(), ((Log) as).color(), ((Log) as).format(), ((Log) as).entryColor());
                e.setPriority(((Log) as).priority());
                break;
            } else if (as instanceof Log.Boolean) {
                e = new BooleanEntry(((Log.Boolean) as).name(), (Supplier<Boolean>) m, ((Log.Boolean) as).index(),
                        ((Log.Boolean) as).trueValue(), ((Log.Boolean) as).falseValue(),
                        ((Log.Boolean) as).color(), ((Log.Boolean) as).trueFormat(),
                        ((Log.Boolean) as).falseFormat(), ((Log.Boolean) as).trueColor(),
                        ((Log.Boolean) as).falseColor());
                e.setPriority(((Log.Boolean) as).priority());
                break;
            } else if (as instanceof LogConfig.Run) {
                init = ((LogConfig.Run) as).duringInit();
                run = ((LogConfig.Run) as).duringRun();
            }

        }
        if (e != null) {
            if (init) {
                unindexedInitEntries.add(e);
            }
            if (run) {
                unindexedRunEntries.add(e);
            }
        }
    }

    /**
     * Repeat a String
     *
     * @param s   The String to repeat
     * @param num The amount of times to repeat the String
     * @return The String s repeated num times
     */
    public static String repeat(String s, int num) {
        return num > 0 ? repeat(s, num - 1) + s : "";
    }

    private static Supplier<?> getCustom(Object o) {
        if (o instanceof Supplier<?>) {
            return (Supplier<?>) o;
        } else if (o instanceof BooleanSupplier) {
            return ((BooleanSupplier) o)::getAsBoolean;
        } else if (o instanceof IntSupplier) {
            return ((IntSupplier) o)::getAsInt;
        } else if (o instanceof DoubleSupplier) {
            return ((DoubleSupplier) o)::getAsDouble;
        } else if(o instanceof Integer){
            return ()-> (Integer) o;
        } else if(o instanceof Double){
            return ()-> (Double) o;
        } else if(o instanceof Boolean){
            return ()-> (Boolean) o;
        } else {
            return null;
        }
    }

    private boolean isFieldAllowed(Field f) {
        if (f.isAnnotationPresent(LogConfig.Whitelist.class)) {
            if (!Arrays.asList(f.getAnnotation(LogConfig.Whitelist.class).value()).contains(opMode.getClass())) {
                return false;
            }
        }
        if (f.isAnnotationPresent(LogConfig.Blacklist.class)) {
            if (Arrays.asList(f.getAnnotation(LogConfig.Blacklist.class).value()).contains(opMode.getClass())) {
                return false;
            }
        }
        return !f.isAnnotationPresent(LogConfig.Disabled.class);
    }
}
