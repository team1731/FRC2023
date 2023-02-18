package frc.robot.state.arm;

public enum ArmEvent {
    // Button Events
    BUTTON_DOWN,
    BUTTON_UP,

    // Subsystem Events
    INITIALIZED,
    COMPLETED_PATH,

    // Parallel Process Events
    POSITIONED
}