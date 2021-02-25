package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants;
import frc.robot.Kinematics;
import frc.robot.Logger;
import frc.robot.OurWPITalonSRX;
import frc.robot.Robot;
import frc.robot.util.*;
import frc.robot.util.geometry.*;

public class Drive extends Subsystem {
  private static Drive mInstance;

  // Drive Talons
  private OurWPITalonSRX mLeftMaster, mRightMaster, mLeftSlave, mRightSlave;

  // Drive is plummed to default to high gear
  private boolean mHighGear = true;

  public enum DriveControlState {
    OPEN_LOOP, // open loop voltage control
    PATH_FOLLOWING, // velocity PID control
  }

  private DriveControlState mDriveControlState;

  private PeriodicDriveData mPeriodicDriveData = new PeriodicDriveData();
  private Logger mDriveLogger;

  public static class PeriodicDriveData {
    // INPUTS
    public double timestamp;
    public double left_voltage;
    public double right_voltage;
    public int left_position_ticks;
    public int right_position_ticks;
    public double left_distance;
    public double right_distance;
    public int left_velocity_ticks_per_100ms;
    public int right_velocity_ticks_per_100ms;
    public Rotation2d gyro_heading = Rotation2d.identity();
    public Pose2d error = Pose2d.identity();
    public boolean climbingSpeed = false;

    // OUTPUTS
    public double left_demand;
    public double right_demand;
    public double left_feedforward;
    public double right_feedforward;
    public double left_old = 0;
    public double right_old = 0;
    public boolean isQuickturning = false;
  }

  public static synchronized Drive getInstance() {
    if (mInstance == null) {
      mInstance = new Drive();
    }
    return mInstance;
  }

  public int feetToTicks(double feet) {
    double ticks;
    if (Robot.getIsPracticeBot()) {
      ticks = Constants.Drive.Encoders.practiceTicksPerFoot;
    } else {
      ticks = Constants.Drive.Encoders.compTicksPerFoot;
    }
    long roundedVal = Math.round(feet * ticks);
    if (roundedVal > Integer.MAX_VALUE) {
      mDriveLogger.warn(
          "Integer overflow when converting feet to ticks! Something is likely VERY WRONG!");
    }

    return (int) roundedVal;
  }

  private Drive() {
    mDriveLogger = new Logger(Constants.Loggers.DRIVE);

    mLeftMaster = new OurWPITalonSRX(Constants.Talons.Drive.leftMaster);
    // configureSpark(mLeftMaster, true, true);

    mLeftSlave = new OurWPITalonSRX(Constants.Talons.Drive.leftSlave);
    // configureSpark(mLeftSlave, true, false);

    mRightMaster = new OurWPITalonSRX(Constants.Talons.Drive.rightMaster);
    // configureSpark(mRightMaster, false, true);

    mRightSlave = new OurWPITalonSRX(Constants.Talons.Drive.rightSlave);
    // configureSpark(mRightSlave, false, false);

    configTalon(mLeftMaster);
    mLeftSlave.setNeutralMode(NeutralMode.Brake);
    if (!Robot.getIsPracticeBot()) mLeftSlave.setSensorPhase(false);

    configTalon(mRightMaster);
    mRightSlave.setNeutralMode(NeutralMode.Brake);

    // Configure slave Talons to follow masters
    mLeftSlave.follow(mLeftMaster);
    mRightSlave.follow(mRightMaster);

    // TODO: figure out what this does and make it work
    setOpenLoop(DriveSignal.NEUTRAL);
  }

  private void configTalon(OurWPITalonSRX talon) {
    talon.configFactoryDefault();
    talon.configNominalOutputForward(0., 0);
    talon.configNominalOutputReverse(0., 0);
    talon.configPeakOutputForward(1, 0);
    talon.configPeakOutputReverse(-1, 0);
    talon.configOpenloopRamp(0);
    talon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    talon.setSensorPhase(true);
    talon.setNeutralMode(NeutralMode.Brake);

    // Configure Talon gains
    double P, I, D;

    P = Constants.Drive.AutoPID.p;
    I = Constants.Drive.AutoPID.i;
    D = Constants.Drive.AutoPID.d;

    mDriveLogger.info("PID values: " + P + ", " + I + ", " + D);

    talon.config_kP(0, P);
    talon.config_kI(0, I);
    talon.config_kD(0, D);
    talon.config_kF(0, 0);
    talon.configClosedLoopPeriod(0, 10);

    talon.configMotionCruiseVelocity(900);
    talon.configMotionAcceleration(750);
  }

  public void resetCruiseAndAccel() {
    setCruiseAndAcceleration(Constants.Auto.defaultCruiseVelocity, Constants.Auto.defaultAccel);
  }

  public void setCruiseAndAcceleration(int cruise, int accel) {
    mLeftMaster.configMotionCruiseVelocity(cruise);
    mRightMaster.configMotionCruiseVelocity(cruise);

    mLeftMaster.configMotionAcceleration(accel);
    mRightMaster.configMotionAcceleration(accel);
  }

  public void configP(double p) {
    mLeftMaster.config_kP(0, p);
    mRightMaster.config_kP(0, p);
  }

  public void configI(double i) {
    mLeftMaster.config_kI(0, i);
    mRightMaster.config_kI(0, i);
  }

  public void configD(double d) {
    mLeftMaster.config_kD(0, d);
    mRightMaster.config_kD(0, d);
  }

  public void resetPID() {
    double P, I, D;

    P = Constants.Drive.AutoPID.p;
    I = Constants.Drive.AutoPID.i;
    D = Constants.Drive.AutoPID.d;

    configP(P);
    configI(I);
    configD(D);
  }

  public void zeroSensors() {
    zeroEncoders();
  }

  public void setMotionMagicTarget(int left, int right) {
    mLeftMaster.set(ControlMode.MotionMagic, left);
    mRightMaster.set(ControlMode.MotionMagic, -right);
  }

  public void setSimplePIDTarget(int left, int right) {
    mLeftMaster.set(ControlMode.Position, left);
    mRightMaster.set(ControlMode.Position, -right);
  }

  /** Configure talons for open loop control */

  // Used for arcade turning during auto
  public void setSimplePercentOutput(DriveSignal signal) {
    mLeftMaster.set(ControlMode.PercentOutput, signal.getLeft());
    mRightMaster.set(ControlMode.PercentOutput, signal.getRight() * -1);
  }

 /**
     * Configure talons for open loop control
     */
    public synchronized void setOpenLoop(DriveSignal signal) {
      if (mDriveControlState != DriveControlState.OPEN_LOOP) {
          //setBrakeMode(true);
          System.out.println("switching to open loop");
          System.out.println(signal);
          mDriveControlState = DriveControlState.OPEN_LOOP;
      }

  

      signal.PrintLog();
      mLeftMaster.set(ControlMode.PercentOutput, signal.getLeft());
  mRightMaster.set(ControlMode.PercentOutput, signal.getRight() * -1);

  }


  public synchronized void setDrive(double throttle, double wheel, boolean quickTurn) {
                  wheel = wheel * -1; //invert wheel
/*
      if(throttle >= 0){
      }
      */

      if (Util.epsilonEquals(throttle, 0.0, 0.04)) {
          throttle = 0.0;
          quickTurn = true;
      }

      if (Util.epsilonEquals(wheel, 0.0, 0.035)) {
          wheel = 0.0;
      }
throttle *= .45;
      final double kWheelGain = 0.05;
      final double kWheelNonlinearity = 0.05;
      final double denominator = Math.sin(Math.PI / 2.0 * kWheelNonlinearity);
      // Apply a sin function that's scaled to make it feel better.
      if (!quickTurn) {
          wheel = Math.sin(Math.PI / 2.0 * kWheelNonlinearity * wheel);
          wheel = Math.sin(Math.PI / 2.0 * kWheelNonlinearity * wheel);
          wheel = wheel / (denominator * denominator) * Math.abs(throttle);
      }

      wheel *= kWheelGain;
      DriveSignal signal = Kinematics.inverseKinematics(new Twist2d(throttle, 0.0, wheel));
      double scaling_factor = Math.max(1.0, Math.max(Math.abs(signal.getLeft()), Math.abs(signal.getRight())));
      setOpenLoop(new DriveSignal(signal.getLeft() / scaling_factor, signal.getRight() / scaling_factor));
  }



  /**
   * WPILib's arcade drive. We need this for auto turning because it allows us to set a rotation
   * speed. Note that the deadband functionality has been removed, since we don't have to worry
   * about driver error during auto. If we were using WPILib's {@link
   * edu.wpi.first.wpilibj.drive.DifferentialDrive DifferentialDrive} we wouldn't need to copy this
   * over. The output is also clamped so that we don't lose control.
   *
   * @param xSpeed The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
   * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is
   *     positive.
   * @param squareInputs If set, decreases the input sensitivity at low speeds.
   */
  public void arcadeHack(double xSpeed, double zRotation, boolean squareInputs) {
    xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);

    zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);

    // Square the inputs (while preserving the sign) to increase fine control
    // while permitting full power.
    if (squareInputs) {
      xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
      zRotation = Math.copySign(zRotation * zRotation, zRotation);
    }

    double leftMotorOutput;
    double rightMotorOutput;

    double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);

    if (xSpeed >= 0.0) {
      // First quadrant, else second quadrant
      if (zRotation >= 0.0) {
        leftMotorOutput = maxInput;
        rightMotorOutput = xSpeed - zRotation;
      } else {
        leftMotorOutput = xSpeed + zRotation;
        rightMotorOutput = maxInput;
      }
    } else {
      // Third quadrant, else fourth quadrant
      if (zRotation >= 0.0) {
        leftMotorOutput = xSpeed + zRotation;
        rightMotorOutput = maxInput;
      } else {
        leftMotorOutput = maxInput;
        rightMotorOutput = xSpeed - zRotation;
      }
    }

    final double max = 0.7;

    setSimplePercentOutput(
        new DriveSignal(
            MathUtil.clamp(leftMotorOutput, -max, max),
            MathUtil.clamp(rightMotorOutput, -max, max)));
  }

  /*
      Test all Sensors in the Subsystem
  */
  public void checkSubsystem() {}

  public synchronized void setClimbingSpeed(boolean climbing) {
    mPeriodicDriveData.climbingSpeed = climbing;
  }

  public synchronized int getLeftEncoderDistance() {
    return mLeftMaster.getSelectedSensorPosition();
  }

  public synchronized int getRightEncoderDistance() {
    return mRightMaster.getSelectedSensorPosition();
  }

  public synchronized Rotation2d getRotation() {
    return null;
  }

  public void zeroEncoders() {
    if (Robot.isReal()) {
      mLeftMaster
          .getSensorCollection()
          .setQuadraturePosition(0, Constants.Drive.talonSensorTimeoutMs);
      mRightMaster
          .getSensorCollection()
          .setQuadraturePosition(0, Constants.Drive.talonSensorTimeoutMs);
    }
  }

  // Used only in TEST mode
  public void setOutputLeftBack(double output) {
    mLeftMaster.set(ControlMode.PercentOutput, output);
  }

  // Used only in TEST mode
  public void setOutputLeftFront(double output) {
    mLeftSlave.set(ControlMode.PercentOutput, output);
  }

  // Used only in TEST mode
  public void setOutputRightBack(double output) {
    mRightMaster.set(ControlMode.PercentOutput, output);
  }

  public synchronized Rotation2d getHeading() {
    return mPeriodicDriveData.gyro_heading;
  }

  // Used only in TEST mode
  public void setOutputRightFront(double output) {
    mRightSlave.set(ControlMode.PercentOutput, output);
  }

  public double getLeftLinearVelocity() {
    return 0;
  }

  public double getRightLinearVelocity() {
    return 0;
  }
}
