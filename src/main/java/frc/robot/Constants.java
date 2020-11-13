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

  public static class Drive {
    public static int talonSensorTimeoutMs = 250;

    public static class Encoders {
      // ticks = (19711 + 19582) / 2
      // distance in feet = 89.5/12
      // ticks per foot = ticks / feet
      private static final double compTicks = (19711.0 + 19582.0) / 2.0;
      private static final double compDistance = 89.5 / 12.0;

      public static double compTicksPerFoot = compTicks / compDistance;
      public static final double practiceTicksPerFoot = 1228.615;
    }

    public static class AutoPID {
      public static final double p = 4;
      public static final double i = 0.00050;
      public static final double d = 0;
    }

    public static class AutoTurnPID {
      public static final double p = 0.2;
      public static final double i = 0;
      public static final double d = 0.015;
      public static final double acceptableError = 0.5;
    }
  }

  public static class TestMode {
    public static final double timePerTest = 1;
    public static final int expectedStorageDistance = 605;
    public static final int acceptableStorageError = 40;
    public static final int expectedShooterSpeed = 970;
    public static final int acceptableShooterError = 150;
  }

  public static class Controllers {
    public static final double joystickDeadband = 0.15;
    public static final double triggerDeadband = 0.15;

    public static class Driver {
      public static final int port = 0;
      public static final String name = "Controller (Xbox One For Windows)";
    }

    public static class Operator {
      public static final int port = 1;
      public static final String name = "AIRFLO";
    }
  }

  // Vision Tracking Constants
  public static class Vision {
    public static final double maxTrackerDistance = 60.0;
    public static final double maxGoalTrackAge = 3.0;
    public static final double maxGoalTrackAgeNotTracking = 0.1;
    public static final double maxGoalTrackSmoothingTime = 0.5;
    public static final double trackStabilityWeight = 0.0;
    public static final double trackAgeWeight = 10.0;
    public static final double trackSwitchingWeight = 100.0;
    public static final double cameraFrameRate = 90.0; // fps
  }

  public static class Turret {
    public static int maxAimVelocity = 80;
    public static final double ticksPerDegree = 140;

    // TODO: determine why we need this
    public static final double manualAdjustFactor = .2;

    public static final double angleOffset = -20;
  }

  public static class Storage {
    // continuous & peak current limit for both storage talons
    public static final int currentLimit = 7;
  }

  public static class Cameras {
    public static final double diagonalView = Math.toRadians(75);
    public static final double horizontalAspect = 4;
    public static final double verticalAspect = 3;
    public static final double diagonalAspect = Math.hypot(horizontalAspect, verticalAspect);
    public static final double horizontalView =
        Math.atan(Math.tan(diagonalView / 2) * (horizontalAspect / diagonalView)) * 2;
    public static final double verticalView =
        Math.atan(Math.tan(diagonalView / 2) * (verticalAspect / diagonalView)) * 2;

    public static class ShooterCamera {
      public static final Rotation2d horizontalPlaneToLens =
          Rotation2d.fromDegrees(0); // Shooter should sit pretty flat
      public static final double height = 40; // shooter camera height on robot (inches)
    }

    public static class BallCamera {
      public static final Rotation2d horizontalPlaneToLens =
          Rotation2d.fromDegrees(-5); // camera is angled downwards
      public static final double height = 12; // ball camera height
    }
  }

  // Field Related Constants
  public static class Field {
    public static final double highGoalHeight = 96.25; // Center Goal Height
    public static final double ballHeight = 5; // ball height (inches)
  }

  public static class RobotDimensions{
    // tested via trial & error, not measured
    // 22 is too low, 100 is too high
    public static final double driveWheelTrackWidthInches = 50;

    public static final double driveWheelDiameterInches = 3.938;
    public static final double driveWheelRadiusInches = driveWheelDiameterInches / 2.0;
    public static final double driveWheelTrackRadiusWidthMeters =
        driveWheelTrackWidthInches / 2.0 * 0.0254;
  }

  public static class Auto{
    public static final int debounceTicks = 10; // ~0.2 seconds
    public static final int defaultAccel = 750;
    public static final int defaultCruiseVelocity = 900;
  }

  // Path Following Constants
  public static final double kPathFollowingMaxAccel = 80.0; // inches per second ^ 2

  // Oracle's official constant convention. Don't @ me.

  public static final double REAL_TRACK_WIDTH = 1.916;


  // Based on how this is used, I'm pretty sure this is a corrective factor
  public static final double kTrackScrubFactor = 1.0469745223;
  public static final double kDriveVoltageRampRate = 0.0;
  public static final int kDriveCurrentThrottledLimit = 30; // amps
  public static final int kDriveCurrentUnThrottledLimit = 80; // amps

  // Offsets from our center point
  public static final Pose2d kTurrentToLens =
      new Pose2d(new Translation2d(0, 0.0), Rotation2d.fromDegrees(0.0));
  public static final Pose2d kWheelsToLens =
      new Pose2d(new Translation2d(0, 0.0), Rotation2d.fromDegrees(0.0));


  // period at which the looper runs at
  public static final double robotLoopPeriod = 0.01;


  /// unused/old
  // Turret
  public static final int kTurretTalonMotorPort = 20;
  public static final double kTurretAimAngleDeadband = .5;

  // PWM
  public static final int kCameraRingId = 0;

  // Constants for Server Motor System
  public static final int kCANTimeoutMs = 10; // use for important on the fly updates
  public static final int kLongCANTimeoutMs = 100; // use for constructor
}
