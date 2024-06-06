package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;


/**
 * Connects the software to the hardware and directly receives data from the gyroscope.
 */
public interface GyroIO {

    /** Contains all of the input data received from hardware. */
    @AutoLog
    public static class GyroIOInputs {
      public Rotation2d heading = new Rotation2d();
    }

    /**
     * Reads information from sources (hardware or simulation) and updates the inputs object.
     *
     * @param inputs Contains the defaults for the input values listed above.
     */
    default void updateInputs(GyroIOInputs inputs) {}
    public default void resetGyro(){}
}