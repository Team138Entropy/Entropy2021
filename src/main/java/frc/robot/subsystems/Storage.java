package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.OurWPITalonSRX;
import frc.robot.Robot;
import javax.annotation.Nullable;

/** Add your docs here. */
public class Storage extends Subsystem {

  private final int ROLLER_BOTTOM_PORT = Constants.Talons.Storage.bottom;
  private final int ROLLER_TOP_PORT;

  private final int STORAGE_CAPICTY = 5;

  private final double STORE_SPEED = Constants.Storage.rollerStoreSpeed;
  private final double TEST_SPEED_FACTOR = Constants.Storage.testRollerSpeedFactor;
  private final double BOTTOM_SPEED_FACTOR = Constants.Storage.bottomRollerSpeedFactor;
  private final double EJECT_SPEED = Constants.Storage.rollerEjectSpeed;
  private final double BALL_DISTANCE_IN_ENCODER_TICKS;
  @Nullable private TimeOfFlight mLidar;
  private final int INTAKE_SENSOR_PORT = 0;

  private DigitalInput mIntakeSensor;

  private final OurWPITalonSRX mBottomRoller;
  private final OurWPITalonSRX mTopRoller;

  private int mBallCount = 0;

  private int mTripCount = 0;

  private static Storage sInstance;

  public static synchronized Storage getInstance() {
    if (sInstance == null) {
      sInstance = new Storage();
    }
    return sInstance;
  }

  public int getCapacity() {
    return STORAGE_CAPICTY;
  }

  private Storage() {
      mLidar = new TimeOfFlight(Constants.Talons.Storage.lidarCanID);
    ROLLER_TOP_PORT =
        Robot.getIsPracticeBot()
            ? Constants.Talons.Storage.practiceTop
            : Constants.Talons.Storage.compTop;

    mBottomRoller = new OurWPITalonSRX(ROLLER_BOTTOM_PORT);
    mTopRoller = new OurWPITalonSRX(ROLLER_TOP_PORT);

    mBottomRoller.configFactoryDefault();
    mTopRoller.configFactoryDefault();

    mTopRoller.setNeutralMode(NeutralMode.Brake);
    mBottomRoller.setNeutralMode(NeutralMode.Brake);

    mTopRoller.configContinuousCurrentLimit(Constants.Storage.currentLimit);
    mTopRoller.configPeakCurrentLimit(Constants.Storage.currentLimit);

    mBottomRoller.configContinuousCurrentLimit(Constants.Storage.currentLimit);
    mBottomRoller.configPeakCurrentLimit(Constants.Storage.currentLimit);

    if (Robot.getIsPracticeBot()) {
      BALL_DISTANCE_IN_ENCODER_TICKS = Constants.Storage.BallDistances.practice;
      mBottomRoller.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
      mBottomRoller.setSensorPhase(false);
    } else {
      BALL_DISTANCE_IN_ENCODER_TICKS = Constants.Storage.BallDistances.competition;
      mTopRoller.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
      mTopRoller.setSensorPhase(false);
      mTopRoller.setInverted(true);
    }

    updateEncoderPosition();
  }

  public synchronized int getEncoder() {
    // return the negative position that the talon gets us because it's hooked up backwards
    // this will return positive values
    if (Robot.getIsPracticeBot()) {
      return mBottomRoller.getSelectedSensorPosition();
    } else {
      return mTopRoller.getSelectedSensorPosition();
    }
  }

  public synchronized void updateEncoderPosition() {
    if (Robot.getIsPracticeBot()) {
      mBottomRoller.setSelectedSensorPosition(0);
    } else {
      mTopRoller.setSelectedSensorPosition(0);
    }
    // mStartingEncoderPosition = getEncoder();
  }

  public synchronized boolean getIntakeSensor() {
    if (Robot.isReal()) {
      return mLidar.getRange() < Constants.Storage.lidarMinDistance;
    }
    return false;
  }

  public synchronized double getSensorDistance() {
    if (Robot.isReal()) {
      return mLidar.getRange();
    }
    return 0;
  }

  public synchronized boolean isBallDetected() {
    if (getIntakeSensor()) {
      mTripCount++;
    } else {
      mTripCount = 0;
    }

    if (mTripCount >= Constants.Storage.minTripCount) {
      mTripCount = 0;
      return true;
    }
    return false;
  }

  public synchronized boolean isBallStored() {
    // this allows us to fit a 5th ball
    if (sInstance.getBallCount() == 4) {
      return true;
    }

    int encoderDistance = getEncoder();
    SmartDashboard.putNumber("Encoder Distance", encoderDistance);
    SmartDashboard.putNumber("Encoder Distance Raw", getEncoder());

    // if we've hit our encoder distance target
    return encoderDistance >= BALL_DISTANCE_IN_ENCODER_TICKS;
  }

  public synchronized void barf() {
    mBottomRoller.set(ControlMode.PercentOutput, -(EJECT_SPEED * BOTTOM_SPEED_FACTOR));
    mTopRoller.set(ControlMode.PercentOutput, -(EJECT_SPEED));
  }

  public synchronized void preloadBalls(int ballCount) {
    mBallCount = ballCount;
  }

  public synchronized void addBall() {
    if (mBallCount < STORAGE_CAPICTY) {
      mBallCount++;
    }
  }

  public synchronized void removeBall() {
    if (mBallCount > 0) {
      mBallCount--;
    }
  }

  public synchronized boolean isEmpty() {
    return mBallCount == 0;
  }

  public synchronized boolean isFull() {
    return mBallCount == STORAGE_CAPICTY;
  }

  public synchronized void storeBall() {
    mBottomRoller.set(ControlMode.PercentOutput, STORE_SPEED * BOTTOM_SPEED_FACTOR);
    mTopRoller.set(ControlMode.PercentOutput, STORE_SPEED);
  }

  public synchronized void emptyBalls() {
    mBallCount = 0;
  }

  /** Stops the roller. */
  public synchronized void stop() {
    mBottomRoller.set(ControlMode.PercentOutput, 0);
    mTopRoller.set(ControlMode.PercentOutput, 0);
  }

  public synchronized void ejectBall() {
    mBottomRoller.set(ControlMode.PercentOutput, EJECT_SPEED * BOTTOM_SPEED_FACTOR);
    mTopRoller.set(ControlMode.PercentOutput, EJECT_SPEED);
  }

  public synchronized void setBottomOutput(double output) {
    mBottomRoller.set(ControlMode.PercentOutput, output * BOTTOM_SPEED_FACTOR * TEST_SPEED_FACTOR);
  }

  public synchronized void setTopOutput(double output) {
    mTopRoller.set(ControlMode.PercentOutput, output * TEST_SPEED_FACTOR);
  }

  public synchronized int getBallCount() {
    return mBallCount;
  }

  @Override
  public void zeroSensors() {}

  @Override
  public void checkSubsystem() {}
}
