package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;

/**
 * Constructs a MAXSwerveModule and configures the driving and turning motor,
 * encoder, and PID controller. This configuration is specific to the REV
 * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
 * Encoder.
 */
public class SwerveModule {

  private final CANSparkMax driveSparkMax;
  private final CANSparkMax turningSparkMax;

  private final RelativeEncoder driveEncoder;
  private final AbsoluteEncoder turningEncoder;

  private final SparkMaxPIDController drivingPidController;
  private final SparkMaxPIDController turningPidController;
  
  private double chassisAngularOffset = 0;
  private SwerveModuleState moduleDesiredState = new SwerveModuleState(0.0, new Rotation2d());

  private int ModuleID;

  public SwerveModule(
       
      int driveMotorId,
      int turningMotorId,
      boolean driveMotorReversed,
      boolean turningMotorReversed,
      double moduleOffset,
      boolean absoluteEncoderReversed) {

    driveSparkMax = new CANSparkMax(driveMotorId, MotorType.kBrushless);
    turningSparkMax = new CANSparkMax(turningMotorId, MotorType.kBrushless);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    driveSparkMax.restoreFactoryDefaults();
    turningSparkMax.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    driveEncoder = driveSparkMax.getEncoder();

    turningEncoder = turningSparkMax.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    turningEncoder.setAverageDepth(16);
    drivingPidController = driveSparkMax.getPIDController();
    turningPidController = turningSparkMax.getPIDController();
    drivingPidController.setFeedbackDevice(driveEncoder);
    turningPidController.setFeedbackDevice(turningEncoder);
    

  //  turningPidController.set
    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
    driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningMotorPositionFactor);
    turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of
    // the steering motor in the MAXSwerve Module.
    turningEncoder.setInverted(absoluteEncoderReversed);

    // Set the PID gains for the driving motor. Note these are example gains, and
    // you
    // may need to tune them for your own robot!
    drivingPidController.setP(ModuleConstants.kDrivingP);
    drivingPidController.setI(ModuleConstants.kDrivingI);
    drivingPidController.setD(ModuleConstants.kDrivingD);
    drivingPidController.setFF(ModuleConstants.kDrivingFF);
    drivingPidController.setOutputRange(ModuleConstants.kDrivingMinInput, ModuleConstants.kDrivingMaxOutput);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    turningPidController.setPositionPIDWrappingEnabled(true);
    turningPidController.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
    turningPidController.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);
    //turningPidController.setSmartMotionAllowedClosedLoopError(50000, 0);

    // Set the PID gains for the turning motor. Note these are example gains, and
    // you
    // may need to tune them for your own robot!
    turningPidController.setP(ModuleConstants.kPTurning);
    turningPidController.setI(ModuleConstants.kITurning);
    turningPidController.setD(ModuleConstants.kDTurning);
    turningPidController.setFF(ModuleConstants.kFFTurning);
    turningPidController.setOutputRange(ModuleConstants.kTurningMinInput, ModuleConstants.kTurningMaxOutput);

    driveSparkMax.setIdleMode(ModuleConstants.kDrivingMotorIdleMode);
    turningSparkMax.setIdleMode(ModuleConstants.kTurningMotorIdleMode);
    driveSparkMax.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
    turningSparkMax.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
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
  public double getTurningPosition() {
   return turningEncoder.getPosition();
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
    //SmartDashboard.putNumber("Module #" + this.ModuleID + " encoder: " + turningEncoder.getPosition());
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
    turningPidController.setReference(correctedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    moduleDesiredState = desiredState;
    SmartDashboard.putNumber("Module #" + this.ModuleID + "desired angle state", desiredState.angle.getRadians());
    SmartDashboard.putNumber("Module #" + this.ModuleID + "\'optimized\' desired angle state ", chassisAngularOffset);
  }

  public void stop() {
    driveSparkMax.set(0);
    turningSparkMax.set(0);
  }
}