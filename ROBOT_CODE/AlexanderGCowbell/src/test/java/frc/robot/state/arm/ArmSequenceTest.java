package frc.robot.state.arm;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import org.junit.jupiter.api.Test;

public class ArmSequenceTest {

  @Test 
  void mapKeyedSequence() {
    ArmSequence sequence = null;

    sequence = ArmSequence.valueForCode("DEPLOY_LOW");
    assertEquals(ArmSequence.SCORE_LOW, sequence, "Should have mapped to SCORE_LOW");

    sequence = ArmSequence.valueForCode("DEPLOY_MID");
    assertEquals(ArmSequence.SCORE_MEDIUM, sequence, "Should have mapped to SCORE_MEDIUM");

    sequence = ArmSequence.valueForCode("DEPLOY_HIGH");
    assertEquals(ArmSequence.SCORE_HIGH, sequence, "Should have mapped to SCORE_HIGH");
  }

  @Test
  void failToMapSequenceWithNoCode() {
    ArmSequence sequence = ArmSequence.valueForCode("INVALID_CODE");
    assertNull(sequence, "Should be NULL (no mapping)");
  }

  @Test
  void setStateMachineKeyedSequence() {
    ArmStateMachine stateMachine = new ArmStateMachine(null);

    stateMachine.setKeyedSequence("CONE_LEFT; DEPLOY_LOW");
    assertEquals(ArmSequence.SCORE_LOW, stateMachine.getKeyedSequence(), "Should have mapped to SCORE_LOW");

    stateMachine.setKeyedSequence("CUBE_MIDDLE; DEPLOY_MID");
    assertEquals(ArmSequence.SCORE_MEDIUM, stateMachine.getKeyedSequence(), "Should have mapped to SCORE_MEDIUM");

    stateMachine.setKeyedSequence("CONE_RIGHT; DEPLOY_HIGH");
    assertEquals(ArmSequence.SCORE_HIGH, stateMachine.getKeyedSequence(), "Should have mapped to SCORE_HIGH");
  }

  @Test
  void stateMachineIgnoresNonScoreKeyedEntry() {
    ArmStateMachine stateMachine = new ArmStateMachine(null);
    stateMachine.setKeyedSequence("INVALID_CODE");
    assertNull(stateMachine.getKeyedSequence(), "Should be NULL (no mapping)");
  }
    
}
