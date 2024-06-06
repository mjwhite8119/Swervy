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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveModule.Place;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and CANcoder
 *
 * <p>
 * NOTE: This implementation should be used as a starting point and adapted to different hardware configurations (e.g. If using an analog encoder, copy from "ModuleIOSparkMax")
 *
 * <p>
 * To calibrate the absolute encoder offsets, point the modules straight (such that forward motion on the drive motor will propel the robot forward) and copy the reported values from the absolute encoders using AdvantageScope. These values are logged under "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOSwervy implements ModuleIO {
  // private final CANcoder cancoder;
  private final Spark driveMotor;
  private final Spark turnMotor;
  private final Encoder driveEncoder;
  private final Encoder turnEncoder;

  private PIDController leftPID =
      new PIDController(0.0, 0.0, 0.0, Constants.loopPeriodSecs);
  private PIDController rightPID =
      new PIDController(0.0, 0.0, 0.0, Constants.loopPeriodSecs);

  private boolean closedLoop = false;
  private double driveFFVolts = 0.0;
  private double turnFFVolts = 0.0;
  private double appliedVoltsDrive = 0.0;
  private double appliedVoltsTurn = 0.0;

  // private final double drivePosition;
  // private final double driveVelocity;
  private final Rotation2d turnAbsoluteInitPosition = new Rotation2d();

  // private final double cancoderAbsolutePosition;
  // private final double turnPosition;
  // private final double turnVelocity;

  // Gear ratios for SDS MK4i L2, adjust as necessary
  private final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0); // 6.746
  private final double TURN_GEAR_RATIO = 150.0 / 7.0; // 21.43
  private final double WHEEL_RADIUS = Constants.Drivetrain.wheelRadius;

  // private final double absoluteEncoderOffset;
  // private final Rotation2d absoluteEncoderOffset;

  // Target Variables. Used only for data logging
  private double targetVelocityMetersPerSeconds = 0;
  private double targetTurnPositionRad = 0;

  public ModuleIOSwervy(Place place) {
    switch(place) {
    case FrontLeft:
      this.turnMotor = new Spark(1);
      this.driveMotor = new Spark(0);
      this.driveEncoder = new Encoder(4, 5);
      this.turnEncoder = new Encoder(6, 7);
      // this.cancoder = new CANcoder(Constants.CAN.swerveFrontdriveEncoder, "canivore");
      // this.absoluteEncoderOffset = Constants.CAN.swerveFrontLeftOffset; // MUST BE CALIBRATED
      break;
    case FrontRight:
      this.turnMotor = new Spark(1);
      this.driveMotor = new Spark(0);
      this.driveEncoder = new Encoder(4, 5);
      this.turnEncoder = new Encoder(6, 7);
      // this.cancoder = new CANcoder(Constants.CAN.swerveFrontturnEncoder, "canivore");
      // this.absoluteEncoderOffset = Constants.CAN.swerveFrontRightOffset; // MUST BE CALIBRATED
      break;
    case BackRight:
      this.turnMotor = new Spark(1);
      this.driveMotor = new Spark(0);
      this.driveEncoder = new Encoder(4, 5);
      this.turnEncoder = new Encoder(6, 7);
      // this.cancoder = new CANcoder(Constants.CAN.swerveBackturnEncoder, "canivore");
      // this.absoluteEncoderOffset = Constants.CAN.swerveBackRightOffset;
      break;
    case BackLeft:
      this.turnMotor = new Spark(1);
      this.driveMotor = new Spark(0);
      this.driveEncoder = new Encoder(4, 5);
      this.turnEncoder = new Encoder(6, 7);
      // this.cancoder = new CANcoder(Constants.CAN.swerveBackdriveEncoder, "canivore");
      // this.absoluteEncoderOffset = Constants.CAN.swerveBackLeftOffset;// MUST BE CALIBRATED
      break;
    default:
      throw new RuntimeException("Invalid module index");
    }

    // -------------------- Config TURN motor ----------------------//

    
    // -------------------- Config DRIVE motor ----------------------//

    

    // -------------------- Config CANcoder  --------------------------// 

    // CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    // encoderConfig.MagnetSensor.MagnetOffset = absoluteEncoderOffset;
    // cancoder.getConfigurator().apply(encoderConfig);

    // ----------------------------------------------------------
    // Inputs
    // ----------------------------------------------------------
    this.driveEncoder.setDistancePerPulse((Math.PI * Constants.kWheelDiameterMeters) / Constants.kCountsPerRevolution);
    this.turnEncoder.setDistancePerPulse((Math.PI * Constants.kWheelDiameterMeters) / Constants.kCountsPerRevolution);
  
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    if (closedLoop) {
      double driveVolts = leftPID.calculate(driveEncoder.getRate()) + driveFFVolts;
      double turnVolts =
          rightPID.calculate(turnEncoder.getRate()) + turnFFVolts;
      appliedVoltsDrive = turnVolts;
      appliedVoltsTurn = driveVolts;
      driveMotor.setVoltage(driveVolts);
      turnMotor.setVoltage(turnVolts * -1);
    }

    inputs.drivePosition = driveEncoder.getDistance();
    inputs.driveVelocity = driveEncoder.getRate();
    inputs.driveAppliedVolts = appliedVoltsDrive;

    inputs.turnPosition = Rotation2d.fromDegrees(turnEncoder.getDistance());
    inputs.turnVelocity = turnEncoder.getRate();
    inputs.turnAppliedVolts = appliedVoltsTurn;

    inputs.cancoderAbsolutePosition =
        new Rotation2d(turnEncoder.getDistance()).plus(turnAbsoluteInitPosition);

  }

  // ----------------------------------------------------------
  // Outputs
  // ----------------------------------------------------------
  @Override
  public void setDriveVoltage(double volts) { 
    closedLoop = false;
    appliedVoltsDrive = volts;
    driveMotor.setVoltage(volts * 1.1);
  }

  @Override
  public void setTurnVoltage(double volts) { 
    closedLoop = false;
    appliedVoltsTurn = volts;
    turnMotor.setVoltage(volts * 1.1);
  }

  @Override
  public void setDriveDutyCycle(double speed) { 
    // driveMotor.setControl(new DutyCycleOut(speed)); 
  }

  @Override
  public void setTurnDutyCycle(double speed) { 
    // turnMotor.setControl(new DutyCycleOut(speed)); 
  }

  @Override
  public void setTargetTurnPosition(double targetTurnPositionRad) {  
    this.targetTurnPositionRad = targetTurnPositionRad;
  }

  @Override
  public void setTargetDriveVelocity(double targetDriveVelocityMetersPerSec) {
    this.targetVelocityMetersPerSeconds = targetDriveVelocityMetersPerSec;
  }

}
