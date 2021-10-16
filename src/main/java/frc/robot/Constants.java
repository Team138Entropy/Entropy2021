// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Logger.SupportedLevels;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public enum Loggers {
    VISION(SupportedLevels.VERBOSE),
    POT(SupportedLevels.DEBUG),
    PID(SupportedLevels.INFO),
    ROBOT(SupportedLevels.WARN),
    TURRET(SupportedLevels.VERBOSE),
    DRIVE(SupportedLevels.INFO),
    SUBSYSTEM(SupportedLevels.INFO),
    BALL_STORED(SupportedLevels.INFO),
    EVENT_WATCHER_THREAD(SupportedLevels.INFO),
    VISION_MANAGER(SupportedLevels.INFO),
    PATH(SupportedLevels.DEBUG),
    OI(SupportedLevels.VERBOSE),
    ROBOT_STATE(SupportedLevels.INFO),
    BALL_DETECTED(SupportedLevels.INFO),
    CAMERA_MANAGER(SupportedLevels.INFO),
    CONFIG(SupportedLevels.INFO),
    STORAGE(SupportedLevels.INFO),
    INTAKE(SupportedLevels.VERBOSE),
    SHOOTER(SupportedLevels.INFO),
    SIMULATEDTALON(SupportedLevels.WARN),
    CLIMBER(SupportedLevels.WARN);

    public SupportedLevels minLevel;

    Loggers(SupportedLevels minLevel) {
      this.minLevel = minLevel;
    }
  }

    public static final class DriveConstants {
        public static final int kLeftMotor1Port = 0;
        public static final int kLeftMotor2Port = 1;
        public static final int kRightMotor1Port = 2;
        public static final int kRightMotor2Port = 3;
    
        public static final int[] kLeftEncoderPorts = new int[] {0, 1};
        public static final int[] kRightEncoderPorts = new int[] {2, 3};
        public static final boolean kLeftEncoderReversed = false;
        public static final boolean kRightEncoderReversed = true;
    
        public static final double kTrackwidthMeters = 0.69;
        public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackwidthMeters);
    
        public static final int kEncoderCPR = 1024;
        public static final double kWheelDiameterMeters = 0.15;
        public static final double kEncoderDistancePerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
            (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;
    
        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or theoretically
        // for *your* robot's drive.
        // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
        // values for your robot.
        public static final double ksVolts = 0.22;
        public static final double kvVoltSecondsPerMeter = 1.98;
        public static final double kaVoltSecondsSquaredPerMeter = 0.2;
    
        // Example value only - as above, this must be tuned for your drive!
        public static final double kPDriveVel = 8.5;
      }
}
