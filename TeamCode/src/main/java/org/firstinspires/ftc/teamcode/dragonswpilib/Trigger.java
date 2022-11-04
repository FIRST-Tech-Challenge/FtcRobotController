package org.firstinspires.ftc.teamcode.dragonswpilib;

public class Trigger {

    //À overrider. Différent de wpilib original.
    public boolean get() {
        return false;
    }

    /**
     * Starts the given command whenever the trigger just becomes active.
     *
     * @param command the command to start
     * @param interruptible whether the command is interruptible
     * @return this trigger, so calls can be chained
     */
    public Trigger whenActive(final Command command, boolean interruptible) {

        CommandScheduler.getInstance()
                .addButton(
                        new Runnable() {
                            private boolean m_pressedLast = get();

                            @Override
                            public void run() {
                                boolean pressed = get();

                                if (!m_pressedLast && pressed) {
                                    command.schedule(interruptible);
                                }

                                m_pressedLast = pressed;
                            }
                        });

        return this;
    }

    /**
     * Starts the given command whenever the trigger just becomes active. The command is set to be
     * interruptible.
     *
     * @param command the command to start
     * @return this trigger, so calls can be chained
     */
    public Trigger whenActive(final Command command) {
        return whenActive(command, true);
    }


    /**
     * Starts the given command when the trigger initially becomes active, and ends it when it becomes
     * inactive, but does not re-start it in-between.
     *
     * @param command the command to start
     * @param interruptible whether the command is interruptible
     * @return this trigger, so calls can be chained
     */
    public Trigger whileActiveOnce(final Command command, boolean interruptible) {

        CommandScheduler.getInstance()
                .addButton(
                        new Runnable() {
                            private boolean m_pressedLast = get();

                            @Override
                            public void run() {
                                boolean pressed = get();

                                if (!m_pressedLast && pressed) {
                                    command.schedule(interruptible);
                                } else if (m_pressedLast && !pressed) {
                                    command.cancel();
                                }

                                m_pressedLast = pressed;
                            }
                        });
        return this;
    }

    /**
     * Toggles a command when the trigger becomes active.
     *
     * @param command the command to togglePince
     * @param interruptible whether the command is interruptible
     * @return this trigger, so calls can be chained
     */
    public Trigger toggleWhenActive(final Command command, boolean interruptible) {

        CommandScheduler.getInstance()
                .addButton(
                        new Runnable() {
                            private boolean m_pressedLast = get();

                            @Override
                            public void run() {
                                boolean pressed = get();

                                if (!m_pressedLast && pressed) {
                                    if (command.isScheduled()) {
                                        command.cancel();
                                    } else {
                                        command.schedule(interruptible);
                                    }
                                }

                                m_pressedLast = pressed;
                            }
                        });
        return this;
    }


    /**
     * Starts the given command when the trigger initially becomes active, and ends it when it becomes
     * inactive, but does not re-start it in-between. The command is set to be interruptible.
     *
     * @param command the command to start
     * @return this trigger, so calls can be chained
     */
    public Trigger whileActiveOnce(final Command command) {
        return whileActiveOnce(command, true);
    }

    /**
     * Toggles a command when the trigger becomes active. The command is set to be interruptible.
     *
     * @param command the command to togglePince
     * @return this trigger, so calls can be chained
     */
    public Trigger toggleWhenActive(final Command command) {
        return toggleWhenActive(command, true);
    }
}
