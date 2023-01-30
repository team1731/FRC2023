package frc.robot.tests;

import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class StateMachineTest {
  
  @BeforeEach // this method will run before each test
  void setup() {
    System.out.println("Starting...");
  }

  @AfterEach // this method will run after each test
  void shutdown() throws Exception {
    System.out.println("Cleaning up...");
  }

  @Test // marks this method as a test
  void test1() {
    System.out.println("Test1...");
  }

  @Test
  void test2() {
    System.out.println("Test2...");
  }
}
