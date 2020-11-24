package frc.robot.auto;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Drive;
import frc.robot.util.DriveSignal;

public class TurnSegment extends Segment {

  private Gyro gyro = Robot.getGyro();
  private Drive drive;

  private double P, I, D;
  private PIDController controller;

  private double degrees;

  // This allows us to only log the encoder positions once every five ticks
  private int loggingCount = 0;

  @SuppressWarnings("FieldCanBeLocal")
  private final int LOGGING_COUNT_MODULUS = 5;

  // The number of ticks for which we've been within the acceptable range
  private int debounceCount = 0;
  private boolean done = false;

  public TurnSegment(double degrees) {
    this.drive = Drive.getInstance();

    degrees = degrees % 360;
    this.degrees = degrees;

    P = Constants.Auto.TurnPID.P;
    I = Constants.Auto.TurnPID.I;
    D = Constants.Auto.TurnPID.D;

    double tolerance = Constants.Drive.AutoTurnPID.acceptableError;

    this.controller = new PIDController(P, I, D);
    this.controller.setSetpoint(degrees);
    this.controller.setTolerance(tolerance);
  }

  @Override
  public void init() {
    logger.info("Initializing turn segment");
    logger.info("Target: " + degrees + "deg");
    gyro.reset();
  }

  @Override
  public void tick() {
    double angle = gyro.getAngle();

    double out = controller.calculate(angle);

    if (++loggingCount > LOGGING_COUNT_MODULUS) {
      logger.info("Angle: " + angle);
      loggingCount = 0;
    }

    drive.arcadeHack(0, out, true);

    if (controller.atSetpoint()) {
      logger.verbose("Angle in acceptable range for " + ++debounceCount + " tick(s)");

      logger.info("Angle: " + angle);

      if (debounceCount >= Constants.Auto.debounceTicks) {
        done = true;
        drive.setOpenLoop(DriveSignal.BRAKE);
      }
    } else {
      debounceCount = 0;
    }
  }

  @Override
  public boolean finished() {
    return done;
  }

  @Override
  public Segment copy() {
    return new TurnSegment(degrees);
  }
}
