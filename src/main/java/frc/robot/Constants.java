package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class Constants {
    private Constants() { throw new IllegalCallerException("Cannot instantiate `Constants`"); }

    public static final double mod(final double lhs, final double rhs) { return (lhs % rhs + rhs) % rhs; }

    public static record PIDValues(
        double p,
        double i,
        double d,
        double f
    ) {
        public final PIDController createController() { return new PIDController(this.p, this.i, this.d); }
    }

    public static record Ratio(double factor) {
        public Ratio(final double from, final double to) { this(to / from); }

        public final double forward(final double value) { return value * this.factor; }
        public final double inverse(final double value) { return value / this.factor; }

        public final Ratio inverse() { return new Ratio(1 / this.factor); }
    }

    public static final class CAN {
        private CAN() { throw new IllegalCallerException("Cannot instantiate `Constants.CAN`"); }

        public static final int pigeon = 0;

        public static final int swerveFrontLeftAzimuth = 16;
        public static final int swerveFrontLeftDrive = 15;
        public static final int swerveFrontLeftEncoder = 16;

        public static final int swerveFrontRightAzimuth = 3;
        public static final int swerveFrontRightDrive = 4;
        public static final int swerveFrontRightEncoder = 3;

        public static final int swerveBackLeftAzimuth = 17;
        public static final int swerveBackLeftDrive = 18;
        public static final int swerveBackLeftEncoder = 17;

        public static final int swerveBackRightAzimuth = 1;
        public static final int swerveBackRightDrive = 2;
        public static final int swerveBackRightEncoder = 1;
    }

    public static final class Drivetrain {
        private Drivetrain() { throw new IllegalCallerException("Cannot instantiate `Constants.Drivetrain`"); }

        // todo: tune
        public static final PIDValues swerveAzimuthPID = new PIDValues(0.3, 0.01, 0.003, 0);
        public static final PIDValues absoluteRotationPID = new PIDValues(0.1, 0, 0, 0);

        public static final double wheelPositionRadius = 0.3906711; // radius of the circle that wheels are positioned on

        public static final Translation2d swerveFrontLeftTranslation = new Translation2d(Constants.Drivetrain.wheelPositionRadius, Rotation2d.fromDegrees(-45));
        public static final Translation2d swerveFrontRightTranslation = new Translation2d(Constants.Drivetrain.wheelPositionRadius, Rotation2d.fromDegrees(45));
        public static final Translation2d swerveBackLeftTranslation = new Translation2d(Constants.Drivetrain.wheelPositionRadius, Rotation2d.fromDegrees(180 + 45));
        public static final Translation2d swerveBackRightTranslation = new Translation2d(Constants.Drivetrain.wheelPositionRadius, Rotation2d.fromDegrees(180 - 45));

        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            Constants.Drivetrain.swerveFrontLeftTranslation,
            Constants.Drivetrain.swerveFrontRightTranslation,
            Constants.Drivetrain.swerveBackLeftTranslation,
            Constants.Drivetrain.swerveBackRightTranslation
        );

        // todo: find
        public static final SimpleMotorFeedforward driveFFW = new SimpleMotorFeedforward(0, 1, 0);

        public static final Ratio motorEncoderToRotations = new Ratio(2048, 1);
        public static final Ratio driveGearMotorToWheel = new Ratio(6.75, 1); // 6.75:1 (motor:wheel)
        public static final Ratio azimuthGearMotorToWheel = new Ratio(150.0 / 7.0, 1); // (150 / 7):1 (motor:wheel)

        public static final double wheelRadius = 2.0 * 0.0254; // m

        // todo: find
        public static final double maxWheelSpeed = 10.0;

        // todo: choose
        public static final double axialLateralSpeed = 1; // m/s
        public static final double thetaSpeed = 180.0; // deg/s
    }
}
