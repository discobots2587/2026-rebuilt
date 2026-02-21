// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.AprilTagPhotonCamera.PhotonCameraConstants;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {

  public static final class DriveConstants {

    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static int year = 2026;

    public static int kFrontLeftDrivingCanId = 30;
    public static int kRearLeftDrivingCanId = 40;
    public static int kFrontRightDrivingCanId = 20;
    public static int kRearRightDrivingCanId = 10;

    public static int kFrontLeftTurningCanId = 31;
    public static int kRearLeftTurningCanId = 41;
    public static int kFrontRightTurningCanId = 21;
    public static int kRearRightTurningCanId = 11;

    static {
      if (year == 2025) {
        kFrontLeftDrivingCanId = 30;
        kRearLeftDrivingCanId = 40;
        kFrontRightDrivingCanId = 20;
        kRearRightDrivingCanId = 10;

        kFrontLeftTurningCanId = 31;
        kRearLeftTurningCanId = 41;
        kFrontRightTurningCanId = 21;
        kRearRightTurningCanId = 11;
      }
    }

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
    public static final double kTriggerButtonThreshold = 0.2;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class IntakeConstants {
    public static final int kIntakeMotorCanId = 55; // change later
    public static final int kIntakeArmMotorCanId = 51; // change later

    public static final class IntakeSetPoints {
      public static final double kIntake = .6; // adjust motor speed later
      public static final double kExtake = -.6; // adjust motor speed later
    }

    public static final class IntakeArmSetPoints {
      public static final double kRaise = .6; // adjust motor speed later
      public static final double kLower = -.6; // adjust motor speed later
    }
  }

  public static final class ShooterSubsystemConstants {
    public static final int kFeederMotorCanId = 5; // SPARK Flex CAN ID
    public static final int kFlywheelMotorCanId = 6; // SPARK Flex CAN ID (Right)
    public static final int kFlywheelFollowerMotorCanId = 7; // SPARK Flex CAN ID (Left)

    public static final class FeederSetpoints {
      public static final double kFeed = 0.95;
    }

    public static final class FlywheelSetpoints {
      public static final double kShootRpm = 5000;
      public static final double kVelocityTolerance = 100;
    }
  }

  public static final class ClimberSubsystemConstants {
    public static final int kClimberMotorCanId = 60; // change later

    public static final class ClimberSetPoints {
      public static final double kClimb = 0.8; // adjust motor speed later
      public static final double kDescend = -0.8; // adjust motor speed later
    }
  }

  public static final class Vision {
    public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, Double.MAX_VALUE);
    public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.1, 0.1, Double.MAX_VALUE);
    public static final Matrix<N3, N1> MULTI_TAG_TELEOP_STD_DEVS = VecBuilder.fill(0.01, 0.01, Double.MAX_VALUE);
    public static final Matrix<N3, N1> SINGLE_TAG_PRECISE_STD_DEVS = VecBuilder.fill(0.05, 0.05, Double.MAX_VALUE);

    public static final PhotonCameraConstants CAMERA_CONSTANTS = new PhotonCameraConstants();
    static {
      CAMERA_CONSTANTS.WIDTH = 1600;
      CAMERA_CONSTANTS.HEIGHT = 1304;
      CAMERA_CONSTANTS.FOV = 95.39;
      CAMERA_CONSTANTS.FPS = 35;
      CAMERA_CONSTANTS.AVG_LATENCY = 30;
      CAMERA_CONSTANTS.STDDEV_LATENCY = 15;
    }
      // Camera names matching the PhotonVision UI
    public static final String[] CAMERA_NAMES = {
      "LeftCamera",
      "RightCamera"
    };

    // Transformations relative to the robot center (Symmetric Front-Left / Front-Right config)
    public static final Transform3d[] ROBOT_TO_CAMS = new Transform3d[] {
      // Left Camera
      new Transform3d(
        new Translation3d(
          Inches.of(6.0).in(Meters),  // X: +6 Front
          Inches.of(9.0).in(Meters),  // Y: +9 Left
          Inches.of(12.5).in(Meters)  // Z: Up
        ),
        new Rotation3d(0, 0, 0)
      ),
      // Right Camera
      new Transform3d(
        new Translation3d(
          Inches.of(6.0).in(Meters),   // X: +6 Front
          Inches.of(-9.0).in(Meters),  // Y: -9 Right (Negative)
          Inches.of(12.5).in(Meters)   // Z: Up
        ),
        new Rotation3d(0, 0, 0)
      ),
    };
  }
}
