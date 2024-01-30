package frc.robot.subsystems;

import java.util.Arrays;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveModule.Place;

public class Drivetrain extends SubsystemBase {
	private final GyroIO gyroIO;
	private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
	private final SwerveModule[] modules = new SwerveModule[4]; // FL, FR, BL, BR
	public final SwerveDriveKinematics kinematics = Constants.Drivetrain.kinematics;

	// Used to track odometry
	public final SwerveDrivePoseEstimator poseEstimator;
	private Pose2d pose = new Pose2d(); 
	private Rotation2d lastGyroRotation = new Rotation2d(); 

	// ----------------------------------------------------------
    // Initialization
    // ----------------------------------------------------------
	public Drivetrain(
		GyroIO gyroIO,
		ModuleIO flModuleIO,
		ModuleIO frModuleIO,
		ModuleIO blModuleIO,
		ModuleIO brModuleIO) 
  	{	
		if (gyroIO == null) gyroIO = new GyroIOSim(this);
		this.gyroIO = gyroIO;
		modules[0] = new SwerveModule(flModuleIO, Place.FrontLeft);
		modules[1] = new SwerveModule(frModuleIO, Place.FrontRight);
		modules[2] = new SwerveModule(blModuleIO, Place.BackLeft);
		modules[3] = new SwerveModule(brModuleIO, Place.BackRight);

		poseEstimator = new SwerveDrivePoseEstimator( this.kinematics,
													this.gyroInputs.yawPosition,
													this.getModulePositions(),
													new Pose2d() 	
													);
	}

	// ----------------------------------------------------------
    // Control Input
    // ----------------------------------------------------------

	/**
	 * Method to drive the robot using joystick info.
	 *
	 * @param xSpeed Speed of the robot in the x direction (forward).
	 * @param ySpeed Speed of the robot in the y direction (sideways).
	 * @param rot Angular rate of the robot.
	 * @param fieldRelative Whether the provided x and y speeds are relative to the field.
	 */
	public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
		SmartDashboard.putNumber("xSpeed", xSpeed);
		SmartDashboard.putNumber("ySpeed", ySpeed);
		SmartDashboard.putNumber("rotation", rot);
		var swerveModuleStates =
			kinematics.toSwerveModuleStates(
				fieldRelative
					? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getHeading())
					: new ChassisSpeeds(xSpeed, ySpeed, rot));

		setModuleStates(swerveModuleStates);
	}

	/**
	 * Set the required speed and angle of each wheel.
	 * 
	 * @param states - required speed in meters per/sec
	 * 				 - angle in degrees
	*/
	public void setModuleStates(final SwerveModuleState[] states) {
		// 5. DESATURATE WHEEL SPEEDS
		SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Drivetrain.maxWheelSpeed);

		// 6. SET SPEED AND ANGLE FOR EACH WHEEL
		for(int i = 0; i < this.modules.length; i++)
			this.modules[i].applyState(states[i]);
	}

	public void setModuleStates(final SwerveModule.State state) { this.setModuleStates(state.states); }

	// Convert field-relative ChassisSpeeds to robot-relative ChassisSpeeds.
	public ChassisSpeeds fieldOrientedDrive(final ChassisSpeeds field) {
		return ChassisSpeeds.fromFieldRelativeSpeeds(field, getHeading().unaryMinus());
	}

	// Compensate for wheel rotation while driving and rotating
	public ChassisSpeeds compensate(final ChassisSpeeds original) {
		// using this function because its just an easy way to rotate the axial/lateral speeds
		return ChassisSpeeds
			.fromFieldRelativeSpeeds(
				original,
				Rotation2d.fromRadians(original.omegaRadiansPerSecond * Constants.Drivetrain.thetaCompensationFactor)
			);
	}
  
	public void resetGyro() {
		gyroIO.resetGyro();
	}

	/**
	 * Resets the odometry to the specified pose.
	 *
	 * @param pose The pose to which to set the odometry.
	 */
	public void resetOdometry(Pose2d pose) {
		this.poseEstimator.resetPosition(
			this.gyroInputs.yawPosition,
			this.getModulePositions(),
			pose);
	}
	
	// ----------------------------------------------------------
    // Control Input
    // ----------------------------------------------------------
	/**
	 * The angle is continuous; that is, it will continue from 360 to
     * 361 degrees. 
	 * 
	 * @return heading of the robot as a Rotation2d.
	 */
	@AutoLogOutput(key = "Gyro/Heading")
	public Rotation2d getHeading() {
		return this.gyroInputs.heading;
	}

	/**
	 * Current reported yaw of the Pigeon2 in degrees
	 * Constructs and returns a Rotation2d
	 * 
	 * @return current angle of the robot
	 */
	@AutoLogOutput(key = "Gyro/YawPosition")
	public Rotation2d getRobotAngle() {
		return this.gyroInputs.yawPosition;
	}

	/**
	 * Takes the negative of the current angular value
	 * and converts given radians to rotations.
	 * 
	 * @return the continuous rotations and partial rotations
	 */
	@AutoLogOutput(key = "Gyro/Rotations")
	public double getGyroRotations() {
		return getHeading().unaryMinus().getRotations();
	}

	/** Returns the current odometry pose. */
	@AutoLogOutput(key = "Odometry/Estimation")
	public Pose2d getPoseEstimation() {
		return this.poseEstimator.getEstimatedPosition();
	}

	public SwerveModule[] getSwerveModules() {return this.modules;}

	public SwerveDriveKinematics getKinematics() {return this.kinematics;}

	// ----------------------------------------------------------
    // Odometry
    // ----------------------------------------------------------
	public SwerveModulePosition[] getModulePositions() {
		return Arrays.stream(this.modules).map(SwerveModule::updateModulePosition).toArray(SwerveModulePosition[]::new);
	}

	/** Returns the current odometry pose. */
	@AutoLogOutput(key = "Odometry/Robot")
	public Pose2d getPose() {
	  	return pose;
	}

	/** Resets the current odometry pose. */
	public void setPose(Pose2d pose) {
		this.pose = pose;
	}

	public void updateOdometry() {
		// Update odometry
		SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
		for (int i = 0; i < 4; i++) {
		wheelDeltas[i] = modules[i].getPositionDelta();
		}
		// The twist represents the motion of the robot since the last
		// loop cycle in x, y, and theta based only on the modules,
		// without the gyro. The gyro is always disconnected in simulation.
		var twist = kinematics.toTwist2d(wheelDeltas);
		if (gyroInputs.connected) {
		// If the gyro is connected, replace the theta component of the twist
		// with the change in angle since the last loop cycle.
		twist =
			new Twist2d(
				twist.dx, twist.dy, gyroInputs.yawPosition.minus(lastGyroRotation).getRadians());
		lastGyroRotation = gyroInputs.yawPosition;
		}
		// Apply the twist (change since last loop cycle) to the current pose
    	pose = pose.exp(twist);
	}

	// ----------------------------------------------------------
    // Process Logic
    // ----------------------------------------------------------
	@Override
	public void periodic() {
		gyroIO.updateInputs(gyroInputs);
		Logger.processInputs("Drive/Gyro", gyroInputs);

		for(final SwerveModule module : this.modules)
			module.update();

		// Stop moving when disabled
		if (DriverStation.isDisabled()) {
			for (var module : modules) {
				module.stop();
			}
		}

		// Log empty setpoint states when disabled
		if (DriverStation.isDisabled()) {
			Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
			Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
			
		}
		
		this.poseEstimator.update(this.gyroInputs.yawPosition, this.getModulePositions());
	}

}
