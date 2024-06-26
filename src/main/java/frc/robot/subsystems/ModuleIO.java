// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs {
    public double drivePosition = 0.0;
    public double driveVelocity = 0.0;
    public double driveAppliedVolts = 0.0;

    public Rotation2d turnPosition = new Rotation2d();
    public double turnVelocity = 0.0;
    public double turnAppliedVolts = 0.0;

    public Rotation2d cancoderAbsolutePosition = new Rotation2d();
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ModuleIOInputs inputs) {}

  /** Run the drive motor at the specified voltage. */
  public default void setDriveVoltage(double volts) {}

  /** Run the turn motor at the specified voltage. */
  public default void setTurnVoltage(double volts) {}

  /** Run the drive motor at the specified duty cycle (-1 to 1). */
  public default void setDriveDutyCycle(double speed) {}

  /** Run the turn motor at the specified duty cycle (-1 to 1). */
  public default void setTurnDutyCycle(double speed) {}

  /** Enable or disable brake mode on the drive motor. */
  public default void setDriveBrakeMode(boolean enable) {}

  /** Enable or disable brake mode on the turn motor. */
  public default void setTurnBrakeMode(boolean enable) {}

  /** Run closed loop position control on the turn motor. */
  public default void setTargetTurnPosition(double targetSteerPositionRad) {}

  /** Run closed loop velocity control on the drive motor. */
  public default void setTargetDriveVelocity(double targetDriveVelocityMetersPerSec) {}

  /** Run closed loop torque control on the drive motor. */
  public default void setTargetDriveTorque(double targetDriveVelocityMetersPerSec) {}
}
