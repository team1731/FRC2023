package frc.robot.state;
 
import java.util.ArrayList; 
import java.util.Iterator;  
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.StateConstants;
import frc.robot.Constants.StateConstants.ResultCode;
import frc.robot.state.StateWaitCondition.WaitType;

public abstract class StateMachine {
  private String id;
  private StateHandler stateHandler;
  private boolean reportExceptions = true; // should only be false during unit testing
  protected Status status;
  protected Iterator<StateChangeRequest> sequence;
  protected State previouState;
  protected State state;
  protected StateChangeRequest currentStep;
  protected ArrayList<StateChange> processedSteps;
  protected ArrayList<Wait> unresolvedWaitConditions;
  protected StateWaitCondition currentWaitCondition;

  public enum Status {
    READY, RUNNING, PAUSED, PINGING, INTERRUPTED, FAILED_INIT, INVALID, FINISHED
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

  public State getState() {
    return state;
  }

  public boolean isInInterruptibleStatus() {
    return (status == Status.RUNNING ||  status == Status.PAUSED || status == Status.PINGING);
  }

  // used for unit testing only to prevent intentionally intiated exceptions from printing out
  public void setReportExceptions(boolean reportExceptions) {
    this.reportExceptions = reportExceptions;
  }

  public ArrayList<StateChange> getProcessedSteps() {
    return processedSteps;
  }

  protected StateHandler getStateHandler() {
    return stateHandler;
  }

  // Called by a command to set up a state sequence and kick-off any pre-sequence checks
  public void initialize(StateSequence selectedSequence) throws StateMachineInitializationException {
    try {
      initializationChecks();

      status = Status.READY;
      currentStep = null;
      processedSteps = new ArrayList<StateChange>();
      unresolvedWaitConditions = new ArrayList<Wait>();
      currentWaitCondition = null;

      // grab and parse the sequence definition
      var sequenceDefinition = defineSequence(selectedSequence);
      var sequenceList = new ArrayList<StateChangeRequest>();
      for(var request : sequenceDefinition) {
        sequenceList.add(request);
        if(request.waitCondition != null) {
          unresolvedWaitConditions.add(request.waitCondition.condition);
        }
      }
      sequence = sequenceList.iterator();
    } catch(StateMachineInitializationException ie) {
      status = Status.FAILED_INIT;
      System.out.println("ERROR: " + id + " failed to initialize");
      throw ie;
    }
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
    // if pinging, then this is the state handler letting us know the result, stop pinging
    if(status == Status.PINGING) {
      status = Status.RUNNING;
      resolveWaitCondition(currentWaitCondition.condition);
      currentWaitCondition = null;
    }

    // transition to next state based on the input provided by the state handler
    // this should be some kind of success/failure input
    previouState = state;
    transitionState(input);
    if(status == Status.INVALID) {
      return; // something went wrong w/transition
    }
    
    processedSteps.add(new StateChange(previouState, state, currentStep, result));

    // if state handler was successful move forward, otherwise, let subclass handle failure condition
    if(result.code == ResultCode.SUCCESS) {
      pickupNextSequenceStep();
    } else {
      handleFailureCondition(result);
    }
  }

  protected void transitionState(Input input) {
    try {
      state = state.next(input);
    } catch(StateMachineInvalidTransitionException ite) {
      if(reportExceptions) {
        DriverStation.reportError(ite.getMessage(), ite.getStackTrace());
      }
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
      if(waitCondition != null && waitCondition.type == WaitType.PAUSE && !checkWaitCondition(waitCondition.condition)) {
        // we have an unresolved PAUSE wait condition, can't process step until this is resolved
        status = Status.PAUSED;
        currentWaitCondition = waitCondition;
      } else if(waitCondition != null && waitCondition.type == WaitType.PING && !checkWaitCondition(waitCondition.condition)) { 
        // we have an unresolved PING wait condition, set the status, but continue processing
        status = Status.PINGING;
        currentWaitCondition = waitCondition;
        processCurrentSequenceStep();
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
    previouState = state;
    transitionState(input);
    if(status == Status.INVALID) {
      return; // something went wrong w/transition
    }

    currentStep.timestamp = Timer.getFPGATimestamp();
    processedSteps.add(new StateChange(previouState, state, currentStep));

    // if pinging we won't call the subsystem yet, periodic will ping subsystem
    if(status != Status.PINGING) {
      // not pinging, go ahead and hand off to the subsystem
      stateHandler.changeState(input, currentStep.data);
    }
  }

  // Called by command running in parallel when it is complete
  public void resolveWaitCondition(Wait waitCondition) {
    if(unresolvedWaitConditions != null) {
      unresolvedWaitConditions.remove(waitCondition);
    }
  }

  private boolean checkWaitCondition(Wait waitCondition) {
    if(unresolvedWaitConditions != null) {
      return unresolvedWaitConditions.indexOf(waitCondition) == -1;
    }

    return true;
  }

  // Called from the initiating command's execute method, which runs every ~20ms
  public void periodic() {
    if(currentWaitCondition != null && 
       currentWaitCondition.type == WaitType.PAUSE && 
       checkWaitCondition(currentWaitCondition.condition)) {
      // this PAUSE wait condition has been resolved, move on and process the paused step
      currentWaitCondition = null;
      status = Status.RUNNING;
      processCurrentSequenceStep();
    }

    if(status == Status.PINGING) {
      stateHandler.periodic(); // ping the subsystem's periodic method
    }
  }
}