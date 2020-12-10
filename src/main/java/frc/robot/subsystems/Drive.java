package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  public enum DriveControlState {
    OPEN_LOOP, // open loop voltage control
    PATH_FOLLOWING, // velocity PID control
  }

  private DriveControlState mDriveControlState;

  private PeriodicDriveData mPeriodicDriveData = new PeriodicDriveData();
  private Logger mDriveLogger;

  private DifferentialDrive mDifferentialDrive = null;

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
    public double velocity;
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

    if (Constants.Drive.driveMode == Constants.Drive.DriveMode.WPILIB_DRIVE) {
      mDifferentialDrive = new DifferentialDrive(mLeftMaster, mRightMaster);
    }
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

  double closestToZero(double num1, double num2) {
    if (Math.abs(num1) < Math.abs(num2)) {
      return num1;
    } else {
      return num2;
    }
  }

  /** Configure talons for open loop control */
  public synchronized void setOpenLoop(DriveSignal signal) {
    // are we quickturning?
    boolean quickturn = mPeriodicDriveData.isQuickturning;

    // Slow down climbing if the climber is extended so we can't rip it off (as easily)
    double peakOutput = Constants.Drive.maxSpeedWhenClimbing;

    if (mPeriodicDriveData.climbingSpeed && !quickturn) {
      mLeftMaster.configPeakOutputForward(peakOutput, 0);
      mLeftMaster.configPeakOutputReverse(-peakOutput, 0);

      mRightMaster.configPeakOutputForward(peakOutput, 0);
      mRightMaster.configPeakOutputReverse(-peakOutput, 0);
    } else {
      mLeftMaster.configPeakOutputForward(1, 0);
      mLeftMaster.configPeakOutputReverse(-1, 0);

      mRightMaster.configPeakOutputForward(1, 0);
      mRightMaster.configPeakOutputReverse(-1, 0);
    }

    // Segments are started by the variables they will need
    boolean leftStationary = false;
    boolean rightStationary = false;

    if (mLeftMaster.getSupplyCurrent() == 0) {
      leftStationary = true;
    }

    if (mRightMaster.getSupplyCurrent() == 0) {
      rightStationary = true;
    }

    // Splitting up var definitions to their unique sections makes the code more readable
    boolean stationary = false;

    if (leftStationary && rightStationary) {
      stationary = true;
    }

    // Don't know why this is here but I'm not gonna remove it
    if (mDriveControlState != DriveControlState.OPEN_LOOP) {
      // setBrakeMode(true);
      mDriveLogger.verbose("switching to open loop " + signal);
      mDriveControlState = DriveControlState.OPEN_LOOP;
    }

    // Cache our signals for more readable code. right is backwards because reasons out of our
    // control
    double leftOutput = signal.getLeft();
    double rightOutput = signal.getRight();

    // boolean noSignal = signal.getLeft() == 0 && signal.getRight() == 0;
    // if (quickturn || (noSignal && stationary)) {
    //   setOpenloopRamp(0);
    // } else {
    //   setOpenloopRamp(Constants.Drive.accelLimit);
    // }

    // pass our outputs
    double[] outputs = limitAccel(leftOutput, rightOutput, signal.getWheel());
    leftOutput = outputs[0];
    rightOutput = outputs[1];

    SmartDashboard.putBoolean("quckturn", quickturn);

    // then we set our master talons, remembering that the physical right of the drivetrain is
    // backwards, for some reason :)
    mLeftMaster.set(ControlMode.PercentOutput, leftOutput);
    mRightMaster.set(ControlMode.PercentOutput, rightOutput * -1);
  }

  /**
     * 
     * @param value1 The first value
     * @param value2 The second value
     * @param tweenAmount The tween amount, 0 returns the first value, 1 returns the second value, .5 returns the average, .25 is closer to the first, etc.
     * @return
     */
    public static double tweenBetween(double value1, double value2, double tweenAmount){
      double difference = value2 - value1;
      return value1 + (difference * tweenAmount);
    }

  private double[] limitAccel(double leftOutput, double rightOutput, double wheel) {
    boolean leftAcceleratingForward = false;
    boolean leftAcceleratingBackwards = false;

    boolean rightAcceleratingForward = false;
    boolean rightAcceleratingBackwards = false;

    if (leftOutput > mPeriodicDriveData.left_old) {
      leftAcceleratingForward = true;
    } else if (leftOutput < mPeriodicDriveData.left_old) {
      leftAcceleratingBackwards = true;
    }

    if (rightOutput > mPeriodicDriveData.right_old) {
      rightAcceleratingForward = true;
    } else if (rightOutput < mPeriodicDriveData.right_old) {
      rightAcceleratingBackwards = true;
    }

    // https://www.chiefdelphi.com/t/shuffleboard-graph-widget-data-updating/165723
    // why does this happen???
    // great job shuffleboard
    double tinyValue = Math.random() / 100000;

    SmartDashboard.putBoolean("leftAcceleratingForward", leftAcceleratingForward);
    SmartDashboard.putBoolean("leftAcceleratingBackwards", leftAcceleratingBackwards);
    SmartDashboard.putBoolean("rightAcceleratingForward", rightAcceleratingForward);
    SmartDashboard.putBoolean("rightAcceleratingBackwards", rightAcceleratingBackwards);
    SmartDashboard.putNumber("left percent output", leftOutput + tinyValue);
    SmartDashboard.putNumber("right percent output", rightOutput + tinyValue);

    SmartDashboard.putBoolean(
        "Brian drive?", Constants.Drive.driveMode == Constants.Drive.DriveMode.BRIAN_DRIVE);

    // acceleration limiting begins here
    if (Constants.Drive.driveMode == Constants.Drive.DriveMode.BRIAN_DRIVE) {
      /*
            Vrobot(t) = [Vleft(t) + Vright(t)] / 2
      The constraint should limit the change in velocity from the prior control input.  We need to scale both inputs
      Delta velocity (t) = | Vrobot(t) - Vrobot(t-1) |
      If delta velocity(t) > delta velocity limit
      Vleft(t) = Vleft(t) * [ delta velocity limit / delta velocity(t) ]
      Vright(t) = Vright(t) * [ delta velocity limit / delta velocity(t) ]
      else if
      Do nothing!
      endif
            */

      // average the two velocities to find the robot velocity (?)
      double robotVelocity = (leftOutput + rightOutput) / 2;

      SmartDashboard.putNumber("robot velocity", robotVelocity + tinyValue);

      // find out how much our velocity changed since last tick
      // positive if we're accelerating, negative if we're deccelerating
      double deltaV = robotVelocity - mPeriodicDriveData.velocity;

      SmartDashboard.putNumber("deltaV", deltaV + tinyValue);

      // we use two different constants for acceleration vs decceleration because our robot's mass
      // is not
      // evenly balanced. we don't need to check the case where deltaV is 0 because we don't need to
      // limit
      // acceleration there (because we're already stable)

      double accelLimitConstant =
          deltaV > 0
              ? Constants.Drive.AccelerationLimiting.acceleration
              : Constants.Drive.AccelerationLimiting.decceleration;

      if (Math.abs(deltaV) > accelLimitConstant) {
        SmartDashboard.putBoolean("accel limited", true);

        mDriveLogger.info(
            "leftOutput " + leftOutput + " * " + (accelLimitConstant / Math.abs(deltaV)));

        leftOutput = leftOutput * (accelLimitConstant / Math.abs(deltaV));
        rightOutput = rightOutput * (accelLimitConstant / Math.abs(deltaV));
        mDriveLogger.info("\tleftOutput " + leftOutput);
      } else {
        SmartDashboard.putBoolean("accel limited", false);
      }

      SmartDashboard.putNumber("left limited percent output", leftOutput + tinyValue);
      SmartDashboard.putNumber("right limited percent output", rightOutput + tinyValue);

      mPeriodicDriveData.velocity = (leftOutput + rightOutput) / 2;
    } else {
      // acceleration limiting works because we limit the amount that the motor percents can
      // increase or decrease between frames (calls to teleopPeriodic())
      // because speed is a factor of motor percent output, this limits the slope of speed
      // and the derivitave (slope) of speed is acceleration

      // note that motor drive is not the same as velocity of that motor, as motors take some time
      // to speed up and slow down. but it's good enough (?) for our purposes

      // speeding up when going forwards is the same acceleration direction as slowing down when
      // reversing
      // and thus has the same constants
      double accelerationLeft = leftOutput - mPeriodicDriveData.left_old;
      double accelerationRight = rightOutput - mPeriodicDriveData.right_old;

      SmartDashboard.putNumber("left accel", accelerationLeft);
      SmartDashboard.putNumber("right accel", accelerationRight);

      SmartDashboard.putBoolean("is left accel limited", false);
      SmartDashboard.putBoolean("is right accel limited", false);
      SmartDashboard.putBoolean("is left decel limited", false);
      SmartDashboard.putBoolean("is right decel limited", false);
      SmartDashboard.putNumber("left limited accel", tinyValue);
      SmartDashboard.putNumber("right limited accel", tinyValue);

      // range of (0, 1)
      // how straight our current heading is, with 1 being the most straight
      double straightness = 1 - Math.abs(wheel * 20);

      double straightnessFactor = 30;

      straightness = straightness / straightnessFactor + (1 - (1 / straightnessFactor));

      // System.out.println(straightness);
      System.out.println(straightness);

      if (leftAcceleratingForward) {
        double leftIncrease = tweenBetween(accelerationLeft, 
            closestToZero(
                accelerationLeft,
                Math.copySign(Constants.Drive.AccelerationLimiting.acceleration, accelerationLeft)), straightness);

        leftOutput = mPeriodicDriveData.left_old + leftIncrease;

        SmartDashboard.putNumber("left limited accel", leftIncrease + tinyValue);
      } else if (leftAcceleratingBackwards) {
        double leftIncrease = tweenBetween(accelerationLeft,
            closestToZero(
                accelerationLeft,
                Math.copySign(
                    Constants.Drive.AccelerationLimiting.decceleration, accelerationLeft)), straightness);

        leftOutput = mPeriodicDriveData.left_old + leftIncrease;

        SmartDashboard.putNumber("left limited accel", leftIncrease + tinyValue);
      }

      if (rightAcceleratingForward) {
        double rightIncrease = tweenBetween(accelerationRight,
            closestToZero(
                accelerationRight,
                Math.copySign(
                    Constants.Drive.AccelerationLimiting.acceleration, accelerationRight)), straightness);

        rightOutput = mPeriodicDriveData.right_old + rightIncrease;

        SmartDashboard.putNumber("right limited accel", rightIncrease + tinyValue);
      } else if (rightAcceleratingBackwards) {
        double rightIncrease = tweenBetween(accelerationRight,
            closestToZero(
                accelerationRight,
                Math.copySign(
                    Constants.Drive.AccelerationLimiting.decceleration, accelerationRight)), straightness);

        rightOutput = mPeriodicDriveData.right_old + rightIncrease;

        SmartDashboard.putNumber("right limited accel", rightIncrease);
      }

      SmartDashboard.putNumber("tinyValue", tinyValue);
      SmartDashboard.putNumber("left limited percent output", leftOutput + tinyValue);
      SmartDashboard.putNumber("right limited percent output", rightOutput + tinyValue);
    }

    // cache our olds after we've used them to make them actually "olds"
    mPeriodicDriveData.left_old = leftOutput;
    mPeriodicDriveData.right_old = rightOutput;

    return new double[] {leftOutput, rightOutput};
  }

  // Used for arcade turning during auto
  public void setSimplePercentOutput(DriveSignal signal) {
    setOpenloopRamp(0); // Just in case
    mLeftMaster.set(ControlMode.PercentOutput, signal.getLeft());
    mRightMaster.set(ControlMode.PercentOutput, signal.getRight() * -1);
  }

  public synchronized void setDrive(double throttle, double wheel) {
    SmartDashboard.putNumber("throttle", throttle);
    SmartDashboard.putNumber("wheel", wheel);
    switch (Constants.Drive.driveMode) {
      case BRIAN_DRIVE:
      case OLD_DRIVE:
        boolean quickTurn = false;
        wheel = wheel * -1; // invert wheel

        // TODO: Extract this "epsilonEquals" pattern into a "handleDeadband" method
        // If we're not pushing forward on the throttle, automatically enable quickturn so that we
        // don't have to
        // explicitly enable it before turning.
        if (Util.epsilonEquals(throttle, 0.0, 0.04)) {
          throttle = 0.0;
          quickTurn = true;
        }

        // This is just a convoluted way to do a deadband.
        if (Util.epsilonEquals(wheel, 0.0, 0.020)) {
          wheel = 0.0;
        }

        if (wheel != 0 && quickTurn) {
          mPeriodicDriveData.isQuickturning = true;
        } else {
          mPeriodicDriveData.isQuickturning = false;
        }

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
        // We pass 0 for dy because we use a differential drive and can't strafe.
        // The wheel here is a constant curvature rather than an actual heading. This is what makes
        // the drive cheesy.
        DriveSignal signal = Kinematics.inverseKinematics(new Twist2d(throttle, 0.0, wheel));

        // Either the bigger of the two drive signals or 1, whichever is bigger.
        double scaling_factor =
            Math.max(
                1.0,
                Math.max(
                    Math.abs(signal.getLeft()),
                    Math.abs(signal.getRight()))); // / (1 + (differenceBetweenSides * 6));

        if (quickTurn) {
          setOpenLoop(
              new DriveSignal(
                  (signal.getLeft() / scaling_factor) / 5,
                  (signal.getRight() / scaling_factor) / 5, wheel));
        } else {
          setOpenLoop(
              new DriveSignal(
                  signal.getLeft() / scaling_factor, signal.getRight() / scaling_factor, wheel));
        }
        break;
      case WPILIB_DRIVE:
        mDifferentialDrive.arcadeDrive(throttle / 1.5, wheel / 1.5);
        mDriveLogger.log(throttle / 5 + " " + wheel / 5);
        break;
    }
  }

  public void setOpenloopRamp(double speed) {
    mLeftMaster.configOpenloopRamp(speed);
    mRightMaster.configOpenloopRamp(speed);
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
            MathUtil.clamp(rightMotorOutput, -max, max), 0));
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
