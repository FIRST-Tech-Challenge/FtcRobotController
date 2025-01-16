// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package TBLib.StateMachine;

import edu.wpi.first.math.Pair;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.util.ArrayList;
import java.util.List;


public class SequenceStateMachine extends Command {
  private final List<Pair<Command, Boolean>> m_states = new ArrayList<>();
  private final List<String> m_statesNames = new ArrayList<>();
  private int m_currentStateIndex = -1;
  private boolean m_runWhenDisabled = true;
  private InterruptionBehavior m_interruptBehavior = InterruptionBehavior.kCancelIncoming;

  
  public SequenceStateMachine(Command... states) {
    addStates(states);
  }

  
  public final void addStates(Command... states) {
    if (m_currentStateIndex != -1) {
      throw new IllegalStateException(
          "States cannot be added to a composition while it's running they mused be pushed");
    }

    CommandScheduler.getInstance().registerComposedCommands(states);

    for (Command state : states) {
      if(m_statesNames.contains(state.getName()))
        throw new IllegalStateException("Two States can't have the same name");
      m_states.add(new Pair<Command,Boolean>(state, false));
      m_statesNames.add(state.getName());
      getRequirements().addAll(state.getRequirements());
      m_runWhenDisabled &= state.runsWhenDisabled();
      if (state.getInterruptionBehavior() == InterruptionBehavior.kCancelSelf) {
        m_interruptBehavior = InterruptionBehavior.kCancelSelf;
      }
    }
  }

  @Override
  public final void initialize() {
    m_currentStateIndex = 0;

    if (!m_states.isEmpty()) {
      m_states.get(0).getFirst().initialize();
    }
  }

  @Override
  public final void execute() {
    if (m_states.isEmpty()) {
      return;
    }

    Command currentState = m_states.get(m_currentStateIndex).getFirst();
    boolean isCurrentPushed = m_states.get(m_currentStateIndex).getSecond();
    currentState.execute();
    if (currentState.isFinished()) {
      currentState.end(false);
      if (isCurrentPushed) {
          m_states.remove(m_currentStateIndex);
          // m_currentStateIndex--;
      }else{
          m_currentStateIndex++;
      }
      if (m_currentStateIndex < m_states.size()) {
        m_states.get(m_currentStateIndex).getFirst().initialize();
      }
    }
  }

  @Override
  public final void end(boolean interrupted) {
    if (interrupted
        && !m_states.isEmpty()
        && m_currentStateIndex > -1
        && m_currentStateIndex < m_states.size()) {
      m_states.get(m_currentStateIndex).getFirst().end(true);
    }
    m_currentStateIndex = -1;
  }

  @Override
  public final boolean isFinished() {
    return m_currentStateIndex == m_states.size();
  }


    //stops the current command and start the new command at param:index.
    //consider this as interrupt to the current command
    public void jumpToState(int index){
      if (m_states.size() <= index)
        throw new IllegalStateException("State doesn't exist index out on range");

      if (m_currentStateIndex > -1){
        m_states.get(m_currentStateIndex).getFirst().end(true);
        if (m_states.get(m_currentStateIndex).getSecond()) {
          m_states.remove(m_currentStateIndex);
        }
      }

      m_currentStateIndex = index;
      m_states.get(m_currentStateIndex).getFirst().initialize();
    }

    public void jumpToState(String stateName){
      int index = m_statesNames.indexOf(stateName);
      if (index == -1)
        throw new IllegalStateException("State doesn't exist");

      if (m_currentStateIndex > -1){
          m_states.get(m_currentStateIndex).getFirst().end(true);
          if (m_states.get(m_currentStateIndex).getSecond()) {
            m_states.remove(m_currentStateIndex);
          }
      }

      m_currentStateIndex = index;
      m_states.get(m_currentStateIndex).getFirst().initialize();
  }

    public int getCurrentStateIndex() {
        return m_currentStateIndex;
    }

    public String getCurrentStateName() {
      return m_states.get(m_currentStateIndex).getFirst().getName();
    }

    public void pushState(Command state){
        if (m_currentStateIndex>-1){
            m_states.add(m_currentStateIndex,new Pair<Command,Boolean>(state, true));
            m_statesNames.add(m_currentStateIndex,state.getName());
        }else{
            m_states.add(new Pair<Command,Boolean>(state, true));
            m_statesNames.add(state.getName());
            m_currentStateIndex = m_states.size()-1;
        }
        m_states.get(m_currentStateIndex).getFirst().initialize();
    }

  @Override
  public boolean runsWhenDisabled() {
    return m_runWhenDisabled;
  }

  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    return m_interruptBehavior;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addIntegerProperty("index", () -> m_currentStateIndex, null);
    builder.addStringProperty("name", () -> m_states.get(m_currentStateIndex).getFirst().getName(), null);
  }
}
