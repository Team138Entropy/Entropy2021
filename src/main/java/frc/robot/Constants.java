package frc.robot;

import frc.robot.util.geometry.Pose2d;
import frc.robot.util.geometry.Rotation2d;
import frc.robot.util.geometry.Translation2d;

// Any Sort of Constant or 'Magic Number' should be defined here
public class Constants {
  // Talons Ports
  // ALL TALON PORTS DEFINED HERE
  public static class Talons {
    public static class Drive {
      public static final int leftMaster = 1;
      public static final int leftSlave = 2;
      public static final int rightMaster = 3;
      public static final int rightSlave = 4;
    }

    public static class Storage {
      public static final int top = 5;
      public static final int bottom = 8;
    }

    public static class Shooter {
      public static final int master = 6;
      public static final int slave = 7;
    }

    public static final int turret = 9;
    public static final int intakeRoller = 10;
    public static final int climber = 11;
  }

  public static class Drive{
    public static class AutoPID{
      public static final double p = 4;
      public static final double i = 0.00050;
      public static final double d = 0;
    }
    public static class AutoTurnPID{
      public static final double p = 0.2;
      public static final double i = 0;
      public static final double d = 0.015;
      public static final double acceptableError = 0.5;
    }
  }

  public static class TestMode{
    public static final double timePerTest = 1;
    public static final int expectedStorageDistance = 605;
    public static final int acceptableStorageError = 40;
    public static final int expectedShooterSpeed = 970;
    public static final int acceptableShooterError = 150;
  }

  public static final double TURRET_MANUAL_ADJUST_FACTOR = .2;

  public static final int STORAGE_CURRENT_LIMIT = 7;

  public static final double kTurretAngleOffset = -20;

  // Controller Ports
  public static final int DriverControllerPort = 0;
  public static final String DriverControllerName = "Controller (Xbox One For Windows)";
  public static final int OperatorControllerPort = 1;
  public static final String OperatorControllerName = "AIRFLO";

  // PWM
  public static final int kCameraRingId = 0;

  // Talon config
  public static int CONFIG_TIMEOUT_MS = 250;

  public static int TURRET_AIM_MAX_VELOCITY = 80;

  // Drive Constants
  public static final double kJoystickThreshold = 0.15;
  // Oracle's official constant convention. Don't @ me.

  // ticks = (19711 + 19582) / 2
  // distance in feet = 89.5/12
  // ticks per foot = ticks / feet

  private static final double COMP_TICKS = (19711.0 + 19582.0) / 2.0;
  private static final double COMP_DISTANCE = 89.5 / 12.0;
  public static double COMP_TICKS_PER_FOOT = COMP_TICKS / COMP_DISTANCE;

  public static final double TICKS_PER_FOOT = 1228.615;
  public static final double REAL_TRACK_WIDTH = 1.916;

  // Auto Constants
  public static final int AUTO_DEBOUNCE_TICKS = 10; // ~0.2 seconds
  public static final int DEFAULT_ACCEL = 750;
  public static final int DEFAULT_CRUISE_VELOCITY = 900;

  // 22 is too low, 100 is too high
  public static final double kDriveWheelTrackWidthInches = 50;

  public static final double kDriveWheelDiameterInches = 3.938;
  public static final double kDriveWheelRadiusInches = kDriveWheelDiameterInches / 2.0;
  public static final double kDriveWheelTrackRadiusWidthMeters =
      kDriveWheelTrackWidthInches / 2.0 * 0.0254;

  // Based on how this is used, I'm pretty sure this is a corrective factor
  public static final double kTrackScrubFactor = 1.0469745223;
  public static final double kDriveVoltageRampRate = 0.0;
  public static final int kDriveCurrentThrottledLimit = 30; // amps
  public static final int kDriveCurrentUnThrottledLimit = 80; // amps

  // Path Following Constants
  public static final double kPathFollowingMaxAccel = 80.0; // inches per second ^ 2

  // Vision Tracking Constants
  public static final double kMaxTrackerDistance = 60.0;
  public static final double kMaxGoalTrackAge = 3.0;
  public static final double kMaxGoalTrackAgeNotTracking = 0.1;
  public static final double kMaxGoalTrackSmoothingTime = 0.5;
  public static final double kTrackStabilityWeight = 0.0;
  public static final double kTrackAgeWeight = 10.0;
  public static final double kTrackSwitchingWeight = 100.0;
  public static final double kCameraFrameRate = 90.0; // fps

  // Camera Specific Information
  public static final double kCameraDiagonalView = Math.toRadians(75);
  public static final double kCameraHorizontalAspect = 4;
  public static final double kCameraVerticalAspect = 3;
  public static final double kDiagonalAspect =
      Math.hypot(kCameraHorizontalAspect, kCameraVerticalAspect);
  public static final double kCameraHorizontalView =
      Math.atan(Math.tan(kCameraDiagonalView / 2) * (kCameraHorizontalAspect / kCameraDiagonalView))
          * 2;
  public static final double kCameraVerticalView =
      Math.atan(Math.tan(kCameraDiagonalView / 2) * (kCameraVerticalAspect / kCameraDiagonalView))
          * 2;
  public static final Rotation2d kShooterCameraHorizontalPlaneToLens =
      Rotation2d.fromDegrees(0); // Shooter should sit pretty flat
  public static final Rotation2d kBallCameraHorizontalPlaneToLens =
      Rotation2d.fromDegrees(-5); // camera is angled downwards
  public static final double kShooterCameraHeight = 40; // shooter camera height on robot (inches)
  public static final double kBallCameraHeight = 12; // ball camera height

  // Offsets from our center point
  public static final Pose2d kTurrentToLens =
      new Pose2d(new Translation2d(0, 0.0), Rotation2d.fromDegrees(0.0));
  public static final Pose2d kWheelsToLens =
      new Pose2d(new Translation2d(0, 0.0), Rotation2d.fromDegrees(0.0));

  // Field Related Constants
  public static final double kHighGoalHeight = 96.25; // Center Goal Height
  public static final double kBallHeight = 5; // ball height (inches)

  // Looper System
  // period at which the looper runs at
  public static final double kLooperDt = 0.01;

  // Constants for Server Motor System
  public static final int kCANTimeoutMs = 10; // use for important on the fly updates
  public static final int kLongCANTimeoutMs = 100; // use for constructor

  // Turret
  public static final int kTurretTalonMotorPort = 20;
  public static final double kTurretAimAngleDeadband = .5;
  public static final double kTicksPerDegee = 140;
}
