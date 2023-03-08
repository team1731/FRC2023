package frc.robot.state.arm;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import org.junit.jupiter.api.Test;

import frc.robot.Constants.OperatorConsoleConstants;

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
  void failToMapSequenceWithInvalidCode() {
    ArmSequence sequence = ArmSequence.valueForCode("INVALID_CODE");
    assertNull(sequence, "Should be NULL (no mapping)");
  }

  @Test
  void setStateMachineKeyedSequence() {
    ArmStateMachine stateMachine = new ArmStateMachine(null);

    stateMachine.setOperatorSequence("CONE_LEFT; DEPLOY_LOW");
    assertEquals(ArmSequence.SCORE_LOW, stateMachine.getOperatorSequence(), "Should have mapped to SCORE_LOW");

    stateMachine.setOperatorSequence("CUBE_MIDDLE; DEPLOY_MID");
    assertEquals(ArmSequence.SCORE_MEDIUM, stateMachine.getOperatorSequence(), "Should have mapped to SCORE_MEDIUM");

    stateMachine.setOperatorSequence("CONE_RIGHT; DEPLOY_HIGH");
    assertEquals(ArmSequence.SCORE_HIGH, stateMachine.getOperatorSequence(), "Should have mapped to SCORE_HIGH");
  }

  @Test
  void stateMachineDefaultsToScoreHigh() {
    ArmStateMachine stateMachine = new ArmStateMachine(null);
    assertEquals(ArmSequence.SCORE_HIGH, stateMachine.getOperatorSequence(), "Should have mapped to SCORE_HIGH");
  }

  @Test
  void stateMachineIgnoresInvalidKeyedEntry() {
    ArmStateMachine stateMachine = new ArmStateMachine(null);
    stateMachine.setOperatorSequence("INVALID_CODE");
    // Note will remain at default value = score high
    assertEquals(ArmSequence.SCORE_HIGH, stateMachine.getOperatorSequence(), "Should have mapped to SCORE_HIGH");
  }

  @Test 
  void mapSwitchSequence() {
    ArmSequence sequence = null;

    sequence = ArmSequence.valueForSwitch(OperatorConsoleConstants.kScoreLowSwitchId);
    assertEquals(ArmSequence.SCORE_LOW, sequence, "Should have mapped to SCORE_LOW");

    sequence = ArmSequence.valueForSwitch(OperatorConsoleConstants.kScoreMediumSwitchId);
    assertEquals(ArmSequence.SCORE_MEDIUM, sequence, "Should have mapped to SCORE_MEDIUM");

    sequence = ArmSequence.valueForSwitch(OperatorConsoleConstants.kScoreHighSwitchId);
    assertEquals(ArmSequence.SCORE_HIGH, sequence, "Should have mapped to SCORE_HIGH");
  }

  @Test
  void failToMapSequenceWithInvalidSwitch() {
    ArmSequence sequence = ArmSequence.valueForSwitch(999);
    assertNull(sequence, "Should be NULL (no mapping)");
  }

  @Test
  void setStateMachineSwitchequence() {
    ArmStateMachine stateMachine = new ArmStateMachine(null);

    stateMachine.setOperatorSequence(OperatorConsoleConstants.kScoreLowSwitchId);
    assertEquals(ArmSequence.SCORE_LOW, stateMachine.getOperatorSequence(), "Should have mapped to SCORE_LOW");

    stateMachine.setOperatorSequence(OperatorConsoleConstants.kScoreMediumSwitchId);
    assertEquals(ArmSequence.SCORE_MEDIUM, stateMachine.getOperatorSequence(), "Should have mapped to SCORE_MEDIUM");

    stateMachine.setOperatorSequence(OperatorConsoleConstants.kScoreHighSwitchId);
    assertEquals(ArmSequence.SCORE_HIGH, stateMachine.getOperatorSequence(), "Should have mapped to SCORE_HIGH");
  }

  @Test
  void stateMachineIgnoresInvalidSwitchEntry() {
    ArmStateMachine stateMachine = new ArmStateMachine(null);
    stateMachine.setOperatorSequence(999);
    // Note will remain at default value = score high
    assertEquals(ArmSequence.SCORE_HIGH, stateMachine.getOperatorSequence(), "Should have mapped to SCORE_HIGH");
  }
    
}
