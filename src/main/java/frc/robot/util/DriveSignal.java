package frc.robot.util;

public class DriveSignal {
  private final double mLeftMotor;
  private final double mRightMotor;
  private final boolean mBrakeMode;
  private final double mWheel;

  public DriveSignal(double left, double right) {
    this(left, right, false, 0);
  }

  public DriveSignal(double left, double right, double wheel) {
    this(left, right, false, wheel);
  }

  public DriveSignal(double left, double right, boolean brakeMode, double wheel) {
    mLeftMotor = left;
    mRightMotor = right;
    mBrakeMode = brakeMode;
    mWheel = wheel;
  }

  public static DriveSignal fromControls(double throttle, double turn, double wheel) {
    return new DriveSignal(throttle - turn, throttle + turn, wheel);
  }

  public static final DriveSignal NEUTRAL = new DriveSignal(0, 0, 0);
  public static final DriveSignal BRAKE = new DriveSignal(0, 0, true, 0);

  public void PrintLog() {}

  public double getLeft() {
    return mLeftMotor;
  }

  public double getRight() {
    return mRightMotor;
  }

  public double getWheel() {
    return mWheel;
  }

  public boolean getBrakeMode() {
    return mBrakeMode;
  }

  @Override
  public String toString() {
    return "L: " + mLeftMotor + ", R: " + mRightMotor + (mBrakeMode ? ", BRAKE" : "");
  }
}
