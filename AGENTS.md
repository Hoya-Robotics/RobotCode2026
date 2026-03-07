# AGENTS.md - Coding Agent Guidelines

This document provides guidelines for AI coding agents working in this FRC robotics codebase.
Team 4152's 2026 robot code built with WPILib, AdvantageKit, and GradleRIO.

## Build Commands

```bash
# Build the project
./gradlew build

# Clean build artifacts
./gradlew clean

# Deploy to RoboRIO
./gradlew deploy

# Run simulator
./gradlew simulateJava

# Format code (auto-runs before compile)
./gradlew spotlessApply

# Check formatting without applying
./gradlew spotlessCheck
```

## Testing

```bash
# Run all tests
./gradlew test

# Run a single test class
./gradlew test --tests "frc.robot.SomeTest"

# Run a specific test method
./gradlew test --tests "frc.robot.SomeTest.methodName"

# Run tests matching a pattern
./gradlew test --tests "*Drive*"
```

Test files go in `src/test/java/` mirroring the main source structure.
Tests use JUnit 5 (Jupiter). Working directory for tests: `build/jni/release`.

## Code Formatting

- **Formatter**: Google Java Format via Spotless
- **Indentation**: 2 spaces
- **Auto-format**: Runs automatically before compilation

Formatting is enforced. Run `./gradlew spotlessApply` if build fails on format check.

## Import Ordering

1. Static imports first: `import static edu.wpi.first.units.Units.*;`
2. Standard library imports
3. Third-party/vendor imports (edu.wpi, com.ctre, com.revrobotics)
4. Project imports (frc.robot.*)

```java
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.subsystems.drive.Drive;
```

## Naming Conventions

| Element           | Convention          | Example                        |
|-------------------|---------------------|--------------------------------|
| Classes           | PascalCase          | `RobotContainer`, `DriveIO`    |
| Interfaces        | PascalCase + IO     | `DriveIO`, `HoodIO`            |
| Methods           | camelCase           | `updateInputs`, `setState`     |
| Variables         | camelCase           | `driveController`, `maxSpeed`  |
| Constants         | camelCase or UPPER  | `maxDriveSpeedMps`, `kP`       |
| Packages          | lowercase           | `frc.robot.subsystems.drive`   |
| Private members   | sometimes `m_`      | `m_autonomousCommand`          |

## Type Conventions

- Use WPILib Units library for physical quantities:
```java
import static edu.wpi.first.units.Units.*;

Distance radius = Inches.of(2.0);
AngularVelocity speed = RotationsPerSecond.of(50.0);
Angle angle = Degrees.of(45.0);
```

- Use `Optional<T>` for nullable return values, never return null
- Annotate IO input classes with `@AutoLog` for AdvantageKit logging

## Project Architecture

### IO Layer Pattern (AdvantageKit)

Every hardware subsystem follows this pattern:
```
SubsystemIO.java       - Interface defining hardware abstraction
SubsystemIOInputs      - @AutoLog class for logged inputs
SubsystemIOHardware    - Real hardware implementation
SubsystemIOSim         - Simulation implementation
Subsystem.java         - Command-based subsystem using IO interface
```

Example:
```java
public interface DriveIO {
  @AutoLog
  class DriveIOInputs {
    public boolean isConnected = false;
    public double velocityMps = 0.0;
  }

  default void updateInputs(DriveIOInputs inputs) {}
  default void setVoltage(double volts) {}
}
```

### State Machine Pattern

Use `StateSubsystem<E>` base class for state-based subsystems:
```java
public class Intake extends StateSubsystem<Intake.State> {
  public enum State { IDLE, INTAKING, EJECTING }

  @Override
  protected void applyState(State state) { /* apply outputs */ }

  @Override
  protected void handleStateTransitions() { /* transition logic */ }
}
```

### Mode-Based Instantiation

Switch implementations based on robot mode in `RobotContainer`:
```java
switch (RobotConfig.getMode()) {
  case SIM -> new Drive(controller, new DriveIOSim(...));
  case REAL -> new Drive(controller, new DriveIOHardware(...));
  case REPLAY -> new Drive(controller, new DriveIO() {});
}
```

## Error Handling

### Hardware Connectivity
Track connection status via motor controller APIs:
```java
inputs.motorConnected = motor.getLastError() == REVLibError.kOk;
inputs.talonConnected = statusSignals.isConnected();
```

### Optional for Nullables
```java
public Optional<Pose2d> getTargetPose() {
  if (target == null) return Optional.empty();
  return Optional.of(target);
}
```

### Default IO Implementations
Interfaces provide empty defaults for simulation/replay:
```java
default void updateInputs(DriveIOInputs inputs) {}
```

### Graceful Degradation
Handle file I/O and sensor failures gracefully, log errors, continue operation.

## Key Files

| File                  | Purpose                                      |
|-----------------------|----------------------------------------------|
| `Robot.java`          | Robot lifecycle, extends LoggedRobot         |
| `RobotContainer.java` | Subsystem instantiation, command bindings    |
| `RobotConfig.java`    | Constants, tunable parameters, robot mode    |
| `RobotState.java`     | Singleton for global state, pose estimation  |
| `FieldConstants.java` | Field geometry, AprilTag layout              |
| `SuperStructure.java` | Coordinates turret subsystems                |

## Vendor Libraries

- **Phoenix 6** (CTRE): TalonFX, CANcoder - use signal synchronization
- **REVLib**: SparkMax, SparkFlex - check `REVLibError` for status
- **AdvantageKit**: Logging, `@AutoLog`, `LoggedTunableNumber`
- **Choreo**: Path planning, `AutoFactory`, trajectory following
- **PhotonLib**: Vision processing, AprilTag detection

## Common Patterns

### Tunable Parameters
```java
private static final LoggedTunableNumber kP =
    new LoggedTunableNumber("Drive/kP", 0.1);
```

### Signal Synchronization (Phoenix)
```java
PhoenixSync.refreshAll(velocitySignal, positionSignal);
```

### Alliance-Aware Coordinates
```java
Pose2d flipped = AllianceFlip.apply(pose);
```

## File License Headers

Most files use WPILib BSD license. Utility files from other teams may have MIT license.
Preserve existing license headers when editing.

## Do NOT

- Return null from methods that could return Optional
- Skip the IO layer pattern for hardware
- Hardcode alliance-specific coordinates without AllianceFlip
- Ignore motor controller error states
- Create new files when editing existing ones suffices
