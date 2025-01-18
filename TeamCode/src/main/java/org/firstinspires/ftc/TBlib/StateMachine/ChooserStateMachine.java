//package TBLib.StateMachine;
//
//import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;
//
//import edu.wpi.first.util.sendable.SendableBuilder;
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.CommandScheduler;
//import org.littletonrobotics.junction.Logger;
//
//import java.util.Map;
//
//public class ChooserStateMachine<K> extends Command{
//
//  private final Map<K, Command> m_states;
//  private K m_selector;
//  private Command m_selectedState;
//  private boolean m_runsWhenDisabled = true;
//  private InterruptionBehavior m_interruptBehavior = InterruptionBehavior.kCancelIncoming;
//
//  private K m_lastSelection = null;
//
//
//  public ChooserStateMachine(Map<K, Command> states, K defaultState) {
//    m_states = requireNonNullParam(states, "states", "ChooserStateMachine");
//    m_selector = requireNonNullParam(defaultState, "default state", "ChooserStateMachine");
//    CommandScheduler.getInstance()
//        .registerComposedCommands(states.values().toArray(new Command[] {}));
//
//    for (Command state : m_states.values()) {
//      getRequirements().addAll(state.getRequirements());
//      m_runsWhenDisabled &= state.runsWhenDisabled();
//      if (state.getInterruptionBehavior() == InterruptionBehavior.kCancelSelf) {
//        m_interruptBehavior = InterruptionBehavior.kCancelSelf;
//      }
//    }
//  }
//
//  public boolean selectionHasChanged(){
//    return this.m_lastSelection == null ||
//            this.m_lastSelection != m_selector;
//  }
//
//  @Override
//  public void initialize() {
//    m_selectedState = m_states.get(m_selector);
//    m_selectedState.initialize();
//    Logger.recordOutput(getName(), m_selectedState.getName());
//    m_lastSelection = m_selector;
//  }
//
//  @Override
//  public void execute() {
//    if (selectionHasChanged()) {
//      m_selectedState.end(true);
//      m_selectedState = m_states.get(m_selector);
//      m_selectedState.initialize();
//      Logger.recordOutput("Choosers/" + getName(), m_selectedState.getName());
//        m_lastSelection = m_selector;
//    }else{
//        m_selectedState.execute();
//        m_lastSelection = m_selector;
//    }
//  }
//
//  @Override
//  public void end(boolean interrupted) {
//    m_selectedState.end(interrupted);
//  }
//
//  @Override
//  public boolean isFinished() {
//    return false;
//  }
//
//  @Override
//  public boolean runsWhenDisabled() {
//    return m_runsWhenDisabled;
//  }
//
//  @Override
//  public InterruptionBehavior getInterruptionBehavior() {
//    return m_interruptBehavior;
//  }
//
//  public void chooseState(K state){
//    this.m_selector = state;
//  }
//
//  @Override
//  public void initSendable(SendableBuilder builder) {
//    super.initSendable(builder);
//
//    builder.addStringProperty(
//        "selected", () -> m_selectedState == null ? "null" : m_selectedState.getName(), null);
//  }
//}
