package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.drivetrain.JoystickDrive;
import frc.robot.commands.drivetrain.LockWheels;
import frc.robot.oi.DriverOI;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.GyroIO;
import frc.robot.subsystems.GyroIORomiGyro;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.ModuleIO;
import frc.robot.subsystems.ModuleIOSwervy;

public class RobotContainer {

	public final LoggedDashboardChooser<Command> autonomousChooser;

	public final DriverOI driverOI = new DriverOI(new CommandXboxController(0));
	
	public final Drivetrain drivetrain;

	public RobotContainer() {

		switch(Constants.currentMode) {
		// Only SIM mode for the Swervy training robot
		case SIM:
			// Real robot, instantiate hardware IO implementations      
			drivetrain = new Drivetrain(
				new GyroIORomiGyro(),
				new ModuleIOSwervy(SwerveModule.Place.FrontLeft),
				new ModuleIOSwervy(SwerveModule.Place.FrontRight),
				new ModuleIOSwervy(SwerveModule.Place.BackLeft),
				new ModuleIOSwervy(SwerveModule.Place.BackRight)
			);
			break;

		default:
			// Replayed robot, disable IO implementations
			drivetrain = new Drivetrain(new GyroIO() {}, 
						new ModuleIO() {}, 
						new ModuleIO() {}, 
						new ModuleIO() {}, 
						new ModuleIO() {});
			break;
		}
		this.autonomousChooser = new LoggedDashboardChooser<>(
			"Autonomous Routine",
			AutonomousRoutines.createAutonomousChooser(this.drivetrain)
		);

		this.configureDriverControls();
	}

	private void configureDriverControls() {
		// this.driverOI.resetFOD.whileTrue(new RunCommand(() -> this.drivetrain.resetGyro())); // Y Button
		// this.driverOI.lock.whileTrue(new LockWheels(this.drivetrain, this.driverOI)); // Left Bumper
	}

	public void teleop() { this.drivetrain.setDefaultCommand(new JoystickDrive(this.drivetrain, this.driverOI)); }

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() { return this.autonomousChooser.get(); }

}
