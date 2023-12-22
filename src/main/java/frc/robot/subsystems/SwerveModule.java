package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.fasterxml.jackson.databind.AnnotationIntrospector.ReferenceProperty.Type;
import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

  private final CANSparkMax driveSparkMax;
  private final CANSparkMax turningSparkMax;

  private final RelativeEncoder driveEncoder;
  private final AbsoluteEncoder turningEncoder;

  private final SparkMaxPIDController drivingPidController;
  private final SparkMaxPIDController turningPidController;


  private double chassisAngularOffset = 0;
  private SwerveModuleState moduleDesiredState = new SwerveModuleState(0.0, new Rotation2d());

  public SwerveModule(
      int driveMotorId,
      int turningMotorId,
      boolean driveMotorReversed,
      boolean turningMotorReversed,
      double moduleOffset,
      boolean absoluteEncoderReversed) {

    driveSparkMax = new CANSparkMax(driveMotorId, MotorType.kBrushless);
    turningSparkMax = new CANSparkMax(turningMotorId, MotorType.kBrushless);

    driveSparkMax.restoreFactoryDefaults();
    turningSparkMax.restoreFactoryDefaults();

    // set up encoders
    driveEncoder = driveSparkMax.getEncoder();
    turningEncoder = turningSparkMax.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    drivingPidController = driveSparkMax.getPIDController();
    turningPidController = turningSparkMax.getPIDController();
    drivingPidController.setFeedbackDevice(driveEncoder);
    turningPidController.setFeedbackDevice(turningEncoder);

    driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
    driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);

    turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningMotorPositionFactor);
    turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

    turningEncoder.setInverted(absoluteEncoderReversed);

    drivingPidController.setP(ModuleConstants.kDrivingP);
    drivingPidController.setI(ModuleConstants.kDrivingI);
    drivingPidController.setD(ModuleConstants.kDrivingD);
    drivingPidController.setFF(ModuleConstants.kDrivingFF);
    drivingPidController.setOutputRange(ModuleConstants.kDrivingMinInput, ModuleConstants.kDrivingMaxOutput);

    turningPidController.setPositionPIDWrappingEnabled(true);
    turningPidController.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
    turningPidController.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);

    turningPidController.setP(ModuleConstants.kPTurning);
    turningPidController.setI(ModuleConstants.kITurning);
    turningPidController.setD(ModuleConstants.kDTurning);
    turningPidController.setFF(ModuleConstants.kFFTurning);
    turningPidController.setOutputRange(ModuleConstants.kTurningMinInput, ModuleConstants.kTurningMaxOutput);

    driveSparkMax.setIdleMode(ModuleConstants.kDrivingMotorIdleMode);
    turningSparkMax.setIdleMode(ModuleConstants.kTurningMotorIdleMode);
    driveSparkMax.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
    turningSparkMax.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

    driveSparkMax.burnFlash();
    turningSparkMax.burnFlash();

    chassisAngularOffset = moduleOffset;
    moduleDesiredState.angle = new Rotation2d(turningEncoder.getPosition());
    driveEncoder.setPosition(0);

  }

  public double getDrivePosition() {
    return driveEncoder.getPosition();
  }

  /**
   * Returns current turn position in range -pi to pi
   */
  /**
   * Returns current turn position in range -pi to pi
   */
  public double getTurningPosition() {
    return turningEncoder.getPosition(); // ModuleConstants.kTurningMotorRotationPerSteerRotation;
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDrivePosition(),
        new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));

  }

  public double getDriveVelocity() {
    return driveEncoder.getVelocity();
  }

  public double getTurningVelocity() {
    return turningEncoder.getVelocity();
  }

  
  public void resetEncoders() {
    driveEncoder.setPosition(0);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(),
        new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(turningEncoder.getPosition()));

    drivingPidController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    turningPidController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

moduleDesiredState = desiredState;
  }

  /*
  state=SwerveModuleState.optimize(state,

  getState().angle);
    driveSparkMax.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    turningSparkMax.set(turningPidController.calculate(getTurningPosition(),
        state.angle.getRadians()));

    SmartDashboard.putNumber("Swerve[" + turningSparkMax.getDeviceId() + "] desired speed", state.speedMetersPerSecond);

    SmartDashboard.putNumber("Swerve[" + turningSparkMax.getDeviceId() + "] desired angle", state.angle.getRadians());

    SmartDashboard.putNumber("Swerve[" + turningSparkMax.getDeviceId() + "] Current Position", getTurningPosition());

    SmartDashboard.putNumber("Swerve[" + turningSparkMax.getDeviceId() + "] encoder Position",
        turningEncoder.getPosition());

    SmartDashboard.putNumber("Swerve[" + turningSparkMax.getDeviceId() + "] absolute encoder Position",
        absoluteEncoder.getPosition());

    SmartDashboard.putNumber("Swerve[" + turningSparkMax.getDeviceId() + "] PID Output",
        turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));

    SmartDashboard.putNumber("Swerve[" + turningSparkMax.getDeviceId() + "] target angle",
        state.angle.getRadians() * ModuleConstants.kTurningMotorRotationPerSteerRotation);

  }
*/

  public void stop() {
    driveSparkMax.set(0);
    turningSparkMax.set(0);
  }
}