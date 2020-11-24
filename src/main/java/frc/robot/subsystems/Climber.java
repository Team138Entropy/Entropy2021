package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Logger;
import frc.robot.OurWPITalonSRX;

public class Climber extends Subsystem {
  private final int PORT_NUMBER = Constants.Talons.climber;

  // TODO: Tune these values
  private final int DETECT_BAND = Constants.Climber.detectionRange;
  private final int EXTENDED_HEIGHT_IN_ENCODER_TICKS = Constants.Climber.Positions.extended;
  private final int RETRACTED_HEIGHT_IN_ENCODER_TICKS = Constants.Climber.Positions.retracted;
  private final double HOMING_SPEED_PERCENT = Constants.Climber.homeSpeed;

  /**
   * Talon SRX/ Victor SPX will support multiple (cascaded) PID loops. For now we just want the
   * primary one.
   */
  private final int PIDLoopIndex = Constants.Climber.pidLoopIndex;

  /** Climber motion command timeout */
  private final int TimeoutMS = Constants.Climber.talonCommandTimeout;

  /** Servo loop gains */
  private final double mMotorKF = Constants.Climber.PID.F;

  private final double mMotorKP = Constants.Climber.PID.P;
  private final double mMotorKI = Constants.Climber.PID.I;
  private final double mMotorKD = Constants.Climber.PID.D;

  /** Aggregation */
  private static Climber sInstance;

  private OurWPITalonSRX mMotor;
  private Logger mLogger;
  private boolean mIsHoming;

  private Climber() {
    mMotor = new OurWPITalonSRX(PORT_NUMBER);
    mLogger = new Logger(Constants.Loggers.CLIMBER);
    mIsHoming = false;
    init();
  }

  public static Climber getInstance() {
    if (sInstance == null) {
      sInstance = new Climber();
    }
    return sInstance;
  }

  public void init() {

    /* Set the peak and nominal outputs, 12V means full */
    mMotor.configNominalOutputForward(0, TimeoutMS);
    mMotor.configNominalOutputReverse(0, TimeoutMS);
    mMotor.configPeakOutputForward(1, TimeoutMS);
    mMotor.configPeakOutputReverse(-1, TimeoutMS);

    mMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, PIDLoopIndex, TimeoutMS);
    mMotor.setSensorPhase(false);
    mMotor.configClosedLoopPeakOutput(0, 1);

    /* set the allowable closed-loop error,
     * Closed-Loop output will be neutral within this range.
     * See Table in Section 17.2.1 for native units per rotation.
     */
    mMotor.configAllowableClosedloopError(0, PIDLoopIndex, TimeoutMS); /* always servo */

    mMotor.config_kF(PIDLoopIndex, mMotorKF, TimeoutMS);
    mMotor.config_kP(PIDLoopIndex, mMotorKP, TimeoutMS);
    mMotor.config_kI(PIDLoopIndex, mMotorKI, TimeoutMS);
    mMotor.config_kD(PIDLoopIndex, mMotorKD, TimeoutMS);

    // Set brake mode to hold at position
    mMotor.setNeutralMode(NeutralMode.Brake);

    // Integral control only applies when the error is small; this avoids integral windup
    mMotor.config_IntegralZone(0, 200, TimeoutMS);
    mMotor.setInverted(true);
  }

  public void extend() {
    mLogger.verbose("Extending climber to " + EXTENDED_HEIGHT_IN_ENCODER_TICKS);
    mMotor.set(ControlMode.Position, EXTENDED_HEIGHT_IN_ENCODER_TICKS);
  }

  public void retract() {
    mLogger.verbose("Retracting the climber to " + RETRACTED_HEIGHT_IN_ENCODER_TICKS);
    mMotor.set(ControlMode.Position, RETRACTED_HEIGHT_IN_ENCODER_TICKS);
  }

  /** Stops the climber */
  public void stop() {
    mLogger.verbose("Stopping the climber");
    mMotor.stopMotor();
  }

  /** Return true if extended */
  public boolean isExtended() {
    int encoderPosition = mMotor.getSelectedSensorPosition();
    return (encoderPosition >= EXTENDED_HEIGHT_IN_ENCODER_TICKS - DETECT_BAND)
        && (encoderPosition <= EXTENDED_HEIGHT_IN_ENCODER_TICKS + DETECT_BAND);
  }

  /** Returns true if retracted */
  public boolean isRetracted() {
    int encoderPosition = mMotor.getSelectedSensorPosition();
    return (encoderPosition >= RETRACTED_HEIGHT_IN_ENCODER_TICKS - Constants.Climber.detectionRange)
        && (encoderPosition
            <= RETRACTED_HEIGHT_IN_ENCODER_TICKS + Constants.Climber.detectionRange);
  }

  /** Jogs the climber */
  public void jog(double speed) {
    // mLogger.verbose("Jogging climber " + speed);
    mMotor.set(ControlMode.PercentOutput, speed);
  }

  public void updateSmartDashboard() {
    SmartDashboard.putNumber("Climber Position", mMotor.getSelectedSensorPosition());
  }

  public int getEncoderPosition() {
    return mMotor.getSelectedSensorPosition();
  }

  public void resetEncoder() {
    mMotor.setSelectedSensorPosition(0);
  }

  public void zeroSensors() {}

  @Override
  public void checkSubsystem() {}
}
