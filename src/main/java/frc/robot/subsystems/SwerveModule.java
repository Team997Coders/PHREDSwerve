package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

  private final CANSparkMax driveMotor;
  public final CANSparkMax turningMotor;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turningEncoder;

  // private final AbsoluteEncoder turningEncoder;

  private final SparkMaxPIDController sparkMaxPid;

  // *********************************************************************
  // *** S h o u l d t h i s b e S p a r k M a x P I D C o n t r o l l e r
  private final PIDController turningPidController;
  // *** ^ ^ ^ ^ ^ ^ ^
  // *** | | | | | | |
  // **********************************************************************

  public final SparkMaxAbsoluteEncoder absoluteEncoder;
  private final boolean absoluteEncoderReversed;
  private final double absoluteEncoderOffsetRad;

  public SwerveModule(
      int driveMotorId,
      int turningMotorId,
      boolean driveMotorReversed,
      boolean turningMotorReversed,
      int absoluteEncoderId, // <--- Missing ??
      double absoluteEncoderOffset,
      boolean absoluteEncoderReversed) {

    this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
    this.absoluteEncoderReversed = absoluteEncoderReversed;

    driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
    turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

    absoluteEncoder = turningMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle); // ??

    driveMotor.setInverted(driveMotorReversed);
    turningMotor.setInverted(turningMotorReversed);

    driveEncoder = driveMotor.getEncoder();
    turningEncoder = turningMotor.getEncoder();

    driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
    driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);

    turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
    turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

    // turningEncoder.setPositionConversionFactor(1);
    // turningEncoder.setVelocityConversionFactor(1);

    turningPidController = new PIDController(
        ModuleConstants.kPTurning,
        ModuleConstants.kITurning,
        ModuleConstants.kDTurning);
    turningPidController.enableContinuousInput(-Math.PI, Math.PI);

    turningPidController.setTolerance(1);
    turningPidController.reset();
    sparkMaxPid = turningMotor.getPIDController();
    sparkMaxPid.setP(ModuleConstants.kPTurning);
    sparkMaxPid.setI(0);
    sparkMaxPid.setD(0);
    sparkMaxPid.setFeedbackDevice(turningEncoder);
    sparkMaxPid.setPositionPIDWrappingEnabled(true);
    sparkMaxPid.setPositionPIDWrappingMaxInput(1);
    sparkMaxPid.setPositionPIDWrappingMinInput(-1);

    resetEncoders();
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
    return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));

  }

  public double getDriveVelocity() {
    return driveEncoder.getVelocity();
  }

  public double getTurningVelocity() {
    return turningEncoder.getVelocity();
  }

  public double getAbsoluteEncoderRad() {
    // get absolute encoder --> value is 0 to 2pi
    // get absolute encoder --> value is 0 to 2pi
    double angle = absoluteEncoder.getPosition();
    angle -= absoluteEncoderOffsetRad; // <-- Added
    // move range to -pi to pi and flip if encoder reversed
    return (angle - Math.PI) * (absoluteEncoderReversed ? -1.0 : 1.0);
  }

  public void resetEncoders() {
    driveEncoder.setPosition(0);
    turningEncoder.setPosition(getAbsoluteEncoderRad());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
  }

  public void setDesiredState(SwerveModuleState state) {
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }

    state = SwerveModuleState.optimize(state, getState().angle);
    driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    turningMotor.set(turningPidController.calculate(getTurningPosition(),
        state.angle.getRadians()));

    SmartDashboard.putNumber("Swerve[" + turningMotor.getDeviceId() + "] desired speed", state.speedMetersPerSecond);

    SmartDashboard.putNumber("Swerve[" + turningMotor.getDeviceId() + "] desired angle", state.angle.getRadians());

    SmartDashboard.putNumber("Swerve[" + turningMotor.getDeviceId() + "] Current Position", getTurningPosition());

    SmartDashboard.putNumber("Swerve[" + turningMotor.getDeviceId() + "] encoder Position",
        turningEncoder.getPosition());

    SmartDashboard.putNumber("Swerve[" + turningMotor.getDeviceId() + "] absolute encoder Position",
        absoluteEncoder.getPosition());

    SmartDashboard.putNumber("Swerve[" + turningMotor.getDeviceId() + "] PID Output",
        turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));

    SmartDashboard.putNumber("Swerve[" + turningMotor.getDeviceId() + "] target angle",
        state.angle.getRadians() * ModuleConstants.kTurningMotorRotationPerSteerRotation);

  }

  public void stop() {
    driveMotor.set(0);
    turningMotor.set(0);
  }
}