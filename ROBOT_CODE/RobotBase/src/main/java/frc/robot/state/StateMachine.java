package frc.robot.state;
 
import java.util.ArrayList; 
import java.util.Iterator;  
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.StateConstants;
import frc.robot.Constants.StateConstants.StateMachineWaitCondition;

public abstract class StateMachine {
  private String id;
  private StateHandler stateHandler;
  protected Status status;
  protected Iterator<StateChangeRequest> sequence;
  protected State state;
  protected StateChangeRequest currentStep;
  protected ArrayList<StateChange> processedSteps;
  protected ArrayList<StateMachineWaitCondition> unresolvedWaitConditions;
  protected StateMachineWaitCondition currentWaitCondition;

  public enum Status {
    READY, RUNNING, WAITING, INVALID, FINISHED
  }

  public StateMachine(String id, StateHandler stateHandler) {
    this.id = id;
    this.stateHandler = stateHandler;
    stateHandler.registerStateMachine(this);
  }

  public String getId() {
    return id;
  }

  public Status getStatus() {
    return status;
  }

  public ArrayList<StateChange> getProcessedSteps() {
    return processedSteps;
  }

  protected StateHandler getStateHandler() {
    return stateHandler;
  }

  // Called by a command to set up a state sequence and kick-off any pre-sequence checks
  public void initialize(StateSequence selectedSequence) throws StateMachineInitializationException {
    initializationChecks();
    status = Status.READY;
    currentStep = null;
    processedSteps = new ArrayList<StateChange>();
    unresolvedWaitConditions = new ArrayList<StateMachineWaitCondition>();
    currentWaitCondition = null;

    // grab and parse the sequence definition
    var sequenceDefinition = defineSequence(selectedSequence);
    var sequenceList = new ArrayList<StateChangeRequest>();
    for(var request : sequenceDefinition) {
      sequenceList.add(request);
      if(request.waitCondition != null) {
        unresolvedWaitConditions.add(request.waitCondition);
      }
    }
    sequence = sequenceList.iterator();
  }

  protected void initializationChecks() throws StateMachineInitializationException {
    // override this method to perform validation during initializeSequence
    // throw initialization exception if checks fail
  }

  // Must override this method to construct the sequence iterator that the state machine will use
  protected abstract StateChangeRequest[] defineSequence(StateSequence selectedSequence) throws StateMachineInitializationException;

  // Called by a command to begin a state sequence
  public void startSequence() {
    status = Status.RUNNING;
    
    if(sequence != null && sequence.hasNext()) {
      pickupNextSequenceStep();
    } else {
      status = Status.FINISHED;
    }
  }

  // Called by a command to interrupt (stop) the state sequence midstream
  public void interruptSequence() {
    // override this method to define how the state machine should handle interruptions
  }

  // Called by the state handler to report success/failure and advance to the next state
  public void transition(Input input, StateChangeResult result) {
    // transition to next state based on the input provided by the state handler
    // this should be some kind of success/failure input
    var previousState = state;
    transitionState(input);
    if(status == Status.INVALID) {
      return; // something went wrong w/transition
    }
    
    processedSteps.add(new StateChange(previousState, state, currentStep, result));

    // if state handler was successful move forward, otherwise, let subclass handle failure condition
    if(result.code.equals(StateConstants.kSuccessCode)) {
      pickupNextSequenceStep();
    } else {
      handleFailureCondition(result);
    }
  }

  protected void transitionState(Input input) {
    try {
      state = state.next(input);
    } catch(StateMachineInvalidTransitionException ite) {
      DriverStation.reportError(ite.getMessage(), ite.getStackTrace());
      status = Status.INVALID;
      handleInvalidTransition();
    }
  }

  // Must override this method to define how the state machine will handle failed conditions
  protected abstract void handleFailureCondition(StateChangeResult result);

  // Must override this method to define how the state machine will handle an invalid transition
  protected abstract void handleInvalidTransition();

  // Called from initialization or transition to pickup the next step in the sequence and process it
  protected void pickupNextSequenceStep() {
    if(sequence.hasNext()) {
      // grab the next step in the sequence and store it
      currentStep = sequence.next();

      // check to see if we have a wait condition that needs to be resolved
      var waitCondition = currentStep.waitCondition;
      if(currentStep.waitCondition != null && !checkWaitCondition(waitCondition)) {
        // we have an unresolved wait condition, can't process step until this is resolved
        status = Status.WAITING;
        currentWaitCondition = waitCondition;
      } else {
        processCurrentSequenceStep();
      }
    } else {
      status = Status.FINISHED;
    }
  }

  // Called from periodic, where we just cleared a wait condition and want to process the provided step
  protected void processCurrentSequenceStep() {
    var input = currentStep.input;
    var previousState = state;
    transitionState(input);
    if(status == Status.INVALID) {
      return; // something went wrong w/transition
    }

    currentStep.timestamp = Timer.getFPGATimestamp();
    processedSteps.add(new StateChange(previousState, state, currentStep));

    // hand off to the subsystem
    stateHandler.changeState(input, currentStep.data);
  }

  // Called by command running in parallel when it is complete
  public void resolveWaitCondition(StateMachineWaitCondition waitCondition) {
    if(unresolvedWaitConditions != null) {
      unresolvedWaitConditions.remove(waitCondition);
    }
  }

  private boolean checkWaitCondition(StateMachineWaitCondition waitCondition) {
    if(unresolvedWaitConditions != null) {
      return unresolvedWaitConditions.indexOf(waitCondition) == -1;
    }

    return true;
  }

  // Called from the initiating command's execute method, which runs every ~20ms
  public void periodic() {
    if(currentWaitCondition != null && checkWaitCondition(currentWaitCondition)) {
      // this wait condition has been resolved, move on
      currentWaitCondition = null;
      status = Status.RUNNING;
      processCurrentSequenceStep();
    }
  }
}