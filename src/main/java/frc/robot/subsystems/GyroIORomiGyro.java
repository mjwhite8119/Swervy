package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.romi.RomiGyro;

public class GyroIORomiGyro implements GyroIO{
  // Set up the RomiGyro
  private final RomiGyro gyro = new RomiGyro();

  // Constructor
  public GyroIORomiGyro() {

  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {       
    inputs.heading = Rotation2d.fromDegrees(gyro.getAngleZ());
  }

  public void resetGyro(){
    gyro.reset();
  }
}
