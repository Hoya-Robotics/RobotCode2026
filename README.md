# "Murphy" - Team 4152 2026 Robot
> Contact software lead at gabriel@nakamoto.ca for any questions

- Won Innovation in Control Award at Durham ON Regional Competition for simulation and logging

### Overview
- 4xCamera global pose estimation (2 Limelight 4s, 2 Photonvision cameras powered by 1 Orange Pi 5) 
- Advantage Kit hardware/simulated I/O interfacing and logging
- MapleSim and WPILIB physically accurate motor simulation and advanced drivetrain sim
- Simple state machine subsystem management
- CTRE Swerve API for optimized odometry thread at higher freq
- Custom auto framework, fuses choreo trajectories and full PID drive to pose for smooth and accurate motion planning ([Autos.java](src/main/java/frc/robot/Autos.java))
- Interpolatable lookup table turret tracking with iterative time of flight shoot on the move, raw doubles for reduced per-cycle object allocation ([TurretCalculator.java](src/main/java/frc/robot/TurretCalculator.java))
- Subsystem current, power and energy logging over time for diagnostics ([BatteryLogger.java](src/main/java/frc/robot/BatteryLogger.java))
- Idiomatic live tunable control gains with automatic hardware I/O ([GenericTunableGains.java](src/main/java/frc/robot/util/GenericTunableGains.java))
