package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;

public final class Constants {

  public static final class ModuleConstants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 1 / 6.75;
    public static int kTurnEncoderTicksPerRotation = 42;
    // public static final double kTurningMotorGearRatio = 1/(150 / 7);
    public static final double kTurningMotorGearRotationPerSteerRotation = 150/7;
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRotationPerSteerRotation * 2 * Math.PI;
    // public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
    public static final double kPTurning = 0.5;
  }

  public static final class DriveConstants {

    // Distance between right and left wheels
    public static final double kTrackWidth = Units.inchesToMeters(25);
    // Distance between front and back wheels
    public static final double kWheelBase = Units.inchesToMeters(25);
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

    public static final int kFrontLeftDriveMotorPort = 11;
    public static final int kBackLeftDriveMotorPort = 41;
    public static final int kFrontRightDriveMotorPort = 21;
    public static final int kBackRightDriveMotorPort = 31;

    public static final int kFrontLeftTurningMotorPort = 12;
    public static final int kBackLeftTurningMotorPort = 42;
    public static final int kFrontRightTurningMotorPort = 22;
    public static final int kBackRightTurningMotorPort = 32;

    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kBackLeftTurningEncoderReversed = false;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kBackRightTurningEncoderReversed = false;

    public static final boolean kFrontLeftDriveEncoderReversed = true;
    public static final boolean kBackLeftDriveEncoderReversed = true;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kBackRightDriveEncoderReversed = false;

    public static final int kFrontLeftDriveAbsoluteEncoderPort = 12;
    public static final int kBackLeftDriveAbsoluteEncoderPort = 42;
    public static final int kFrontRightDriveAbsoluteEncoderPort = 22;
    public static final int kBackRightDriveAbsoluteEncoderPort = 32;

    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

    public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 2*Math.PI-4.937;
    public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 2*Math.PI-5.814;
    public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 2*Math.PI-4.754;
    public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 2*Math.PI-1.224;

    public static final double kPhysicalMaxSpeedMetersPerSecond = 3;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
        kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
    public static final double kMaxAngularSpeedRadiansPerSecond = //
        DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
    public static final double kPXController = 1.5;
    public static final double kPYController = 1.5;
    public static final double kPThetaController = 3;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond,
            kMaxAngularAccelerationRadiansPerSecondSquared);
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;

    public static final int kDriverYAxis = Axis.kLeftY.value;
    public static final int kDriverXAxis = Axis.kLeftX.value;
    public static final int kDriverRotAxis = Axis.kRightX.value;
    public static final int kDriverFieldOrientedButtonIdx = Button.kA.value;

    public static final int kZeroHeadingBtn = Button.kLeftBumper.value;
    public static final int kXButton = Button.kX.value;
    public static final int kYButton = Button.kY.value;

    public static final double kDeadband = 0.2;
  }
}