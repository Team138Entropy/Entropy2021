package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI.OperatorInterface;
import frc.robot.auto.IntakeSegment;
import frc.robot.auto.Path;
import frc.robot.auto.Paths;
import frc.robot.auto.ShootSegment;
import frc.robot.auto.SyncIntakeSegment;
import frc.robot.auto.VisionToggleSegment;
import frc.robot.subsystems.*;
import frc.robot.util.LatchedBoolean;
import frc.robot.util.loops.Looper;
import frc.robot.vision.VisionPacket;
import edu.wpi.first.wpilibj.Jaguar;
import java.io.IOException;
import java.lang.reflect.Array;
import java.net.InetAddress;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

/**
 * The VM is configured to automatically run this class. If you change the name of this class or the
 * package after creating this project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot {

  // Modes
  public enum Mode {
    Sharpshooter,
    Rebounder,
    Climber
  }

  // State variables
  public enum State {
    IDLE, // Default state
    INTAKE,
    SHOOTING,
    CLIMBING
  }

  public enum IntakeState {
    IDLE, // Default state when State is not INTAKE
    READY_TO_INTAKE,
    INTAKE_WAITING,
    INTAKE,
    STORE_BALL,
    STORAGE_COMPLETE,
    STORAGE_EJECT
  }

  public enum ShootingState {
    IDLE, // Default state when State is not SHOOTING
    PREPARE_TO_SHOOT,
    SHOOT_BALL,
    SHOOT_BALL_COMPLETE,
    SHOOTING_COMPLETE
  }

  public enum ClimbingState {
    IDLE, // Default state when State is not CLIMBING
    WAIT,
    EXTENDING,
    EXTENDING_COMPLETE,
    HOLD,
    RETRACTING,
    RETRACTING_COMPLETE
  }

  public enum TurretState {
    AUTO_AIM,
    MANUAL
  }

  public enum DriveState {
    MANUAL,
    AUTO_DRIVE
  }

  // Vision Related Valuables
  private double LastDistance = -1;
  private int LastTurretVisionID = -1; // use IDs to filter out bad ideas

  private double mTurretAdjust = 0;

   

  public enum TestState {
    START,
    TEST_PI,
    TEST_LIGHT,
    INTAKE_FORWARD,
    INTAKE_BACKWARD,
    STORAGE_ENCODER_FORWARDS_TEST,
    STORAGE_ENCODER_FORWARDS_TEST_WAITING,
    STORAGE_ENCODER_STOP,
    STORAGE_ENCODER_BACKWARDS_TEST,
    STORAGE_ENCODER_BACKWARDS_TEST_WAITING,
    STORAGE_ENCODER_NO_ENCODER_FORWARDS_TEST,
    STORAGE_ENCODER_NO_ENCODER_BACKWARDS_TEST,
    SHOOTER_ENCODER_TEST,
    SHOOTER_ENCODER_TEST_WAITING,
    DRIVE_LEFT_FRONT,
    DRIVE_LEFT_BACK,
    DRIVE_RIGHT_FRONT,
    DRIVE_RIGHT_BACK,
    MANUAL
  }

  private boolean mIsPracticeBot = true;

  private final int AUTONOMOUS_BALL_COUNT = 3;
  private final int BARF_TIMER_DURATION = 3;

  private final int ShotCooldown = 15; // each loop is 20 secs.. 200 ms cooldown
  private int mCurrentCooldown = ShotCooldown;

  private State mState = State.IDLE;
  private IntakeState mIntakeState = IntakeState.IDLE;
  private ShootingState mShootingState = ShootingState.IDLE;
  private ClimbingState mClimbingState = ClimbingState.IDLE;
  private TurretState mTurretState = TurretState.MANUAL;
  private DriveState mDriveState = DriveState.MANUAL;
  private TestState mTestState;

  // Auto stuff
  private static boolean mAuto = false;
  private boolean mShooterIsStopped = false;
  private Path mAutoPath = Paths.NO_OP;

  // Controller Reference
  private final OperatorInterface mOperatorInterface = OperatorInterface.getInstance();

  // Subsystem Manager
  private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();

  // Subsystems
  private final VisionManager mVisionManager = VisionManager.getInstance();
  private final Shooter mShooter = Shooter.getInstance();
  private final Intake mIntake = Intake.getInstance();
  private final Storage mStorage = Storage.getInstance();
  private final Climber mClimber = Climber.getInstance();
  private final Turret mTurret = Turret.getInstance();
  private final Drive mDrive = Drive.getInstance();

  private static final DigitalInput practiceInput = new DigitalInput(Constants.practiceJumperPin);

  private static boolean isPracticeBot = !practiceInput.get();

  // Looper - Running on a set period
  private final Looper mEnabledLooper = new Looper(Constants.robotLoopPeriod);

  private CameraManager mCameraManager;

  private final RobotTracker mRobotTracker = RobotTracker.getInstance();

  /**
   * The robot's gyro. Don't use this for absolute measurements. See {@link #getGyro()} for more
   * details.
   *
   * @see #getGyro()
   */
  private static ADXRS450_Gyro sGyro = new ADXRS450_Gyro();

  public Relay visionLight = new Relay(0);

  static NetworkTable mTable;

  private static boolean sIsSpinningUp = false;

  private Timer mBarfTimer = new Timer();

  Logger mRobotLogger = new Logger(Constants.Loggers.ROBOT);

  // Shooter velocity trim state
  LatchedBoolean mShooterVelocityTrimUp = new LatchedBoolean();
  LatchedBoolean mShooterVelocityTrimDown = new LatchedBoolean();

  private int mTestPosition;
  private Timer mTestTimer = new Timer();

  private Jaguar jag0, jag1, jag2, jag3, jag4, jag5;
  List<Jaguar> allJags = new ArrayList<Jaguar>();
  int allMotors;
  double jagSpeed = 0; 
  private int selectedMotor = 0;
  
  



  //private final NykoController OperatorController;

  @Override

  public void robotInit() {
    allJags.add(jag0 = new Jaguar(0));
    allJags.add(jag1 = new Jaguar(1));
    allJags.add(jag2 = new Jaguar(2));
    allJags.add(jag3 = new Jaguar(3));
    allJags.add(jag4 = new Jaguar(4));
    allJags.add(jag5 = new Jaguar(5));

    for(int i = 0; i < allJags.size(); i++ ){
      allJags.get(i).set(0);
    }

    allMotors = allJags.size() + 1;

    

    //SmartDashboard.putNumber("Auto Layout", 0);
    SmartDashboard.putBoolean("Correct Controllers", mOperatorInterface.checkControllers());

    /*
    // Read the jumper pin for practice bot
    readIsPracticeBot();

    // Register the Enabled Looper
    // Used to run background tasks!
    // Constantly collects information
    mSubsystemManager.registerEnabledLoops(mEnabledLooper);

    // Zero all necessary sensors on Robot
    mSubsystemManager.zeroSensors();

    // Reset Robot Tracker - Note starting position of the Robot
    // This starting Rotation, X, Y is now the Zero Point
    mRobotTracker.reset();

    // prepare the network table
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    mTable = inst.getTable("SmartDashboard");

    mCameraManager = CameraManager.getInstance();
    mCameraManager.init();

    // Set the initial Robot State
    mState = State.INTAKE;
    mIntakeState = IntakeState.IDLE;
    mClimbingState = ClimbingState.IDLE;
    mShootingState = ShootingState.IDLE;

    if (Constants.BallIndicator.enabled) {
      BallIndicator.getInstance();
    }

    sGyro.calibrate();
    */
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putBoolean("Correct Controllers", mOperatorInterface.checkControllers());
  }

  private void updateSmartDashboard() {

    mClimber.updateSmartDashboard();

    SmartDashboard.putBoolean(
        "Intake Mode",
        mState == State.INTAKE
            && mIntakeState != IntakeState.IDLE
            && mIntakeState != IntakeState.READY_TO_INTAKE);
    SmartDashboard.putBoolean("Shooting Mode", mState == State.SHOOTING);
    SmartDashboard.putBoolean("Climbing Mode", mState == State.CLIMBING);

    SmartDashboard.putBoolean("Practice Bot", getIsPracticeBot());
    SmartDashboard.putString("Turret State", mTurretState.toString());

    SmartDashboard.putBoolean("Manual Spin-up", sIsSpinningUp);
    SmartDashboard.putBoolean("Correct Controllers", mOperatorInterface.checkControllers());
    /*
     * SmartDashboard.putBoolean("Has Vision", result.HasResult); if
     * (result.HasResult) { SmartDashboard.putNumber("Turret Offset Error",
     * -result.turret_error.getDegrees()); } else {
     * SmartDashboard.putNumber("Turret Offset Error", 0); }
     */
    SmartDashboard.putNumber("Ball Counter", mStorage.getBallCount());
    SmartDashboard.putBoolean("ShooterFull", mStorage.isFull());
    SmartDashboard.putBoolean("ShooterSpunUp", mShooter.isAtVelocity());
    SmartDashboard.putNumber("ElevateTrim", mShooter.getVelocityAdjustment());

    SmartDashboard.putString("RobotState", mState.name());
    SmartDashboard.putString("IntakeState", mIntakeState.name());
    SmartDashboard.putString("ShootingState", mShootingState.name());
    SmartDashboard.putString("ClimbingState", mClimbingState.name());

    SmartDashboard.putBoolean("Garage Door", mStorage.getIntakeSensor());
    SmartDashboard.putNumber("Shooter Speed", mShooter.getSpeed());

    SmartDashboard.putNumber("Vision Distance", LastDistance);
    SmartDashboard.putBoolean("Has Vision", LastDistance != -1);

    SmartDashboard.putNumber("Has Intake Sensor", mStorage.getIntakeSensor() ? 1 : 0);
    SmartDashboard.putNumber("Raw Intake Sensor", mStorage.getSensorDistance());
  }

  @Override
  public void autonomousInit() {
    mShooter.stop();
    int autoMode = (int) Math.round(SmartDashboard.getNumber("Auto Layout", 1));

    mAuto = true;
    sIsSpinningUp = false;
    mOperatorInterface.checkControllers();

    mRobotLogger.log("Auto Init Called " + autoMode);

    // Start background looper
    // collections information periodically
    mEnabledLooper.start();

    mState = State.SHOOTING;
    mShootingState = ShootingState.IDLE;
    mIntakeState = IntakeState.IDLE;
    mStorage.preloadBalls(AUTONOMOUS_BALL_COUNT);

    mAutoPath = Paths.find("comp4").orElse(Paths.NO_OP);
    mShooterIsStopped = false;
    IntakeSegment
        .resetActivatedState(); // In case we didn't cleanly finish for some reason (emergency
    // stop?)
    ShootSegment.resetState();
  }

  @Override
  public void autonomousPeriodic() {
    mAutoPath.tick();

    if (ShootSegment.shouldStartShooting()) {
      mRobotLogger.info("Setting shooting state to prepare to shoot");
      mShootingState = ShootingState.PREPARE_TO_SHOOT;
    }

    if (mStorage.isEmpty() && !mShooterIsStopped) {
      mShooterIsStopped = true;
      mRobotLogger.info("Setting shooting state to complete");
      mShootingState = ShootingState.SHOOTING_COMPLETE;
      mStorage.stop();
    }

    if (IntakeSegment.isActive() || SyncIntakeSegment.isActive()) {
      executeIntakeStateMachine();
    }

    if (VisionToggleSegment.getToggle()) {
      toggleVision();
    }

    if (sIsSpinningUp) {
      mShooter.start();
    } else if (mState != State.SHOOTING) {
      mShooter.stop();
    }

    executeShootingStateMachine();

    updateSmartDashboard();

    turretLoop();
  }

  @Override
  public void teleopInit() {
    mClimber.resetEncoder();

    visionLight.set(Relay.Value.kOff);
    mAuto = false;
    sIsSpinningUp = false;
    mRobotLogger.log("Teleop Init!");

    // Start background looper
    // collections information periodically
    mEnabledLooper.start();

    mOperatorInterface.checkControllers();

    // Set the initial Robot State
    mState = State.INTAKE;
    mIntakeState = IntakeState.IDLE;
    mClimbingState = ClimbingState.IDLE;
    mShootingState = ShootingState.IDLE;

    mDrive.zeroEncoders();

    // updated in Intake.java
    SmartDashboard.putBoolean("Intake Spinning Up", false);
    SmartDashboard.putBoolean("Intake Overcurrent", false);
    SmartDashboard.putBoolean("Intake Overcurrent Debounced", false);
    SmartDashboard.putNumber("Intake Current", 0);
    SmartDashboard.putNumber("Intake Current Countdown", 0);
    SmartDashboard.putNumber("Encoder Distance", 0);
    SmartDashboard.putNumber("Encoder Distance Raw", 0);
  }

  @Override
  public void teleopPeriodic() {

    int left = mDrive.getLeftEncoderDistance();
    int right = -mDrive.getRightEncoderDistance();
    SmartDashboard.putNumber("Left", left);
    SmartDashboard.putNumber("Right", right);

    try {
      RobotLoop();
    } catch (Exception e) {
      mRobotLogger.log("RobotLoop Exception: " + e.getMessage());

      // print the exception to the system error
      e.printStackTrace(System.err);
    }
  }

  @Override
  public void testInit() {
    mAuto = false;
    mTestState = TestState.MANUAL;
    mSubsystemManager.checkSubsystems();

    SmartDashboard.putBoolean("Raspberry PI Passed", false);
    SmartDashboard.putBoolean("Intake Forwards Passed", false);
    SmartDashboard.putBoolean("Intake Backwards Passed", false);
    SmartDashboard.putBoolean("Storage Forwards Test Passed", false);
    SmartDashboard.putBoolean("Storage Backwards Test Passed", false);
    SmartDashboard.putBoolean("Storage No Encoder Forwards Test Passed", false);
    SmartDashboard.putBoolean("Storage No Encoder Backwards Test Passed", false);
    SmartDashboard.putBoolean("Shooter Initial Speed Test Passed", false);
    SmartDashboard.putBoolean("Shooter Speed Test Passed", false);
    SmartDashboard.putBoolean("Drive Left Front Passed", false);
    SmartDashboard.putBoolean("Drive Left Back Passed", false);
    SmartDashboard.putBoolean("Drive Right Front Passed", false);
    SmartDashboard.putBoolean("Drive Right Back Passed", false);
  }

  interface JustAnEncoder {
    default int getEncoder() {
      return 0;
    }
  }

  interface MotorWithEncoder {
    void percentOutput(double output);

    default int getEncoder() {
      return 0;
    }
  }

  private void setupMotorTest(JustAnEncoder func) {
    mTestPosition = func.getEncoder();

    mTestTimer.reset();
    mTestTimer.start();
  }

  private boolean runMotorTest(
      MotorWithEncoder func,
      String name,
      boolean hasEncoder,
      int expectedPosition,
      int acceptableError,
      double testTime) {

    func.percentOutput(1d);

    if (mTestTimer.get() >= testTime) {
      if (hasEncoder) {
        int deltaPosition = func.getEncoder() - mTestPosition;
        int error = Math.abs(deltaPosition - expectedPosition);
        SmartDashboard.putNumber(name + " Delta", deltaPosition);
        SmartDashboard.putNumber(name + " Error", error);
        if (error > acceptableError) {
          SmartDashboard.putBoolean(name + " Passed", false);
        } else {
          SmartDashboard.putBoolean(name + " Passed", true);
        }
      } else {
        SmartDashboard.putBoolean(name + " Passed", true);
      }

      func.percentOutput(0d);

      mTestTimer.reset();
      mTestTimer.start();

      return true;
    }
    return false;
  }

  @Override
  public void testPeriodic() {

  }

  @Override
  public void disabledInit() {
    

    visionLight.set(Relay.Value.kOff);

    // zero the encoder
    mTurret.zeroSensors();

    // zero turret sensor
    // this assumes the turret is aligned

    mOperatorInterface.resetOverride();
    mClimber.resetEncoder();
  }

  @Override
  public void disabledPeriodic() {}

  // turret loop
  // constantly commands the turret with vision or manual controls
  public synchronized void turretLoop() {
    SmartDashboard.putNumber("Vision Turret Adjust", mTurretAdjust);
    if (mTurretState == TurretState.AUTO_AIM) {
      double turretAdjust = mOperatorInterface.getTurretAdjust();
      mTurretAdjust += turretAdjust * Constants.Turret.manualAdjustFactor;

      // Command the Turret with vision set points
      // RobotTracker.RobotTrackerResult result =
      // mRobotTracker.GetTurretError(Timer.getFPGATimestamp());
      // double dis = result.distance;
      VisionPacket vp = mRobotTracker.GetTurretVisionPacket(Timer.getFPGATimestamp());
      if (vp.HasValue == true) {
        // We have Target Information
        LastDistance = vp.Distance;
        SmartDashboard.putNumber("vision.distance", LastDistance);
        SmartDashboard.putNumber("vision.rawAngle", vp.Error_Angle);
        SmartDashboard.putNumber("vision.turretOffset", vp.getTurretOffset());

        // verify we haven't already commanded this packet!
        if (vp.ID != LastTurretVisionID) {
          mTurret.SetAimError(vp.Error_Angle + (vp.getTurretOffset() * -1) + mTurretAdjust);
          LastTurretVisionID = vp.ID;
        }

      } else {
        // No Results, Don't Rotate

      }
    } else {
      // Command the Turret Manually
      // Operator Controls
      double ManualTurn = mOperatorInterface.getTurretAdjust();
      if (Math.abs(ManualTurn) > .2) {
        mTurret.SetManualOutput(mOperatorInterface.getTurretAdjust());
      } else {
        mTurret.SetManualOutput(0);
      }
    }
  }

  public void driveTrainLoop() {
    // TODO: Cache whether or not the robot has a drivetrain. We shouldn't be calling the config
    // system every tick.
    if (Constants.Drive.enabled) {
      // Check User Inputs
      double driveThrottle = mOperatorInterface.getDriveThrottle();
      double driveTurn = mOperatorInterface.getDriveTurn();

      boolean WantsAutoAim = mOperatorInterface.getFeederSteer();

      // Continue Driving
      if (WantsAutoAim) {
        // Harvest Mode - AutoSteer Functionality
        // Used for tracking a ball
        // we may want to limit the speed?
        // RobotTracker.RobotTrackerResult DriveResult =
        // mRobotTracker.GetFeederStationError(Timer.getFPGATimestamp());
        mDriveState = DriveState.AUTO_DRIVE;

        VisionPacket vp = mRobotTracker.GetTurretVisionPacket(Timer.getFPGATimestamp());
        // mDrive.autoSteerFeederStation(driveThrottle, vp.Error_Angle);
      } else {
        // Standard Manual Drive
        
        //mDrive.setDrive(driveThrottle, driveTurn, false);

        // if we were previously in auto drive.. turn it off
        if (mDriveState == DriveState.AUTO_DRIVE) {
          mDriveState = DriveState.MANUAL;
        }
      }
    }
  }



  public void toggleVision() {
    if (mTurretState == TurretState.AUTO_AIM) {
      // Turn off Auto Aiming
      visionLight.set(Relay.Value.kOff);
      mTurretState = TurretState.MANUAL;
    } else if (mTurretState == TurretState.MANUAL) {
      // Turn on Auto Aiming
      mTurretState = TurretState.AUTO_AIM;

      // Enable Light
      visionLight.set(Relay.Value.kForward);
    }
  }

  /*
    Called constantly, houses the main functionality of robot
  */
  public void RobotLoop() {

    //Select the next motor
    if(mOperatorInterface.getShooterVelocityTrimUp()){
      selectedMotor = selectedMotor+1;
      if(selectedMotor > allMotors){
        selectedMotor = 0;
      }
      if(selectedMotor != allMotors){
        System.out.println("Selected motor is:" + selectedMotor);
      }
      else{
        System.out.println("Controlling all motors");
      }
    }
    //Select the previous motor
    else if(mOperatorInterface.getShooterVelocityTrimDown()){
      selectedMotor = selectedMotor-1;
      if(selectedMotor < 0){
        selectedMotor = allMotors;
      }
      if(selectedMotor != allMotors){
        System.out.println("Selected motor is:" + selectedMotor);
      }
      else{
        System.out.println("Controlling all motors");
      }
    }

    if(mOperatorInterface.getSpinUp()){
      jagSpeed = jagSpeed + .1;
      System.out.println(jagSpeed);

      if(selectedMotor == allMotors){
        System.out.println("Speed up all motors");
        for(int i = 0; i < allJags.size(); i++){
          allJags.get(i).set(jagSpeed);
        }
      }
      else if(selectedMotor != 7){
        allJags.get(selectedMotor).set(jagSpeed);
      }
    }

    if(mOperatorInterface.getShoot()){
      jagSpeed = jagSpeed - .1;
      if(selectedMotor == allMotors){
        System.out.println("Slow all motors");
        for(int i = 0; i < allJags.size(); i++){
          allJags.get(i).set(jagSpeed);
        }
      }
      else if(selectedMotor != 7){
        allJags.get(selectedMotor).set(jagSpeed);
      }
      System.out.println(jagSpeed);
    }

    if(mOperatorInterface.isStorageRollerBottomTest()){
      System.out.println("Stop all motors");
      jagSpeed = 0;
      for(int i = 0; i < allJags.size(); i++){
        allJags.get(i).set(jagSpeed);
      }
      System.out.println(jagSpeed);
    }

    /*
    if (mOperatorInterface.getVisionToggle()) {
      toggleVision();
    }

    mShooter.updateDistance(LastDistance);
    turretLoop();
    driveTrainLoop();

    updateSmartDashboard();
    */

    /*
    State prevState = mState;
    IntakeState prevIntakeState = mIntakeState;
    ClimbingState prevClimbState = mClimbingState;
    ShootingState prevShootState = mShootingState;
    executeRobotStateMachine();
    if (prevState != mState) {
      mRobotLogger.log("Changed state to " + mState);
    }
    if (prevIntakeState != mIntakeState) {
      mRobotLogger.log("Changed state to " + mIntakeState);
    }
    if (prevClimbState != mClimbingState) {
      mRobotLogger.log("Changed state to " + mClimbingState);
    }
    if (prevShootState != mShootingState) {
      mRobotLogger.log("Changed state to " + mShootingState);
    }

    if (mOperatorInterface.getStateReset()) {
      mState = State.INTAKE;
      mIntakeState = IntakeState.IDLE;
      mClimbingState = ClimbingState.IDLE;
      mShootingState = ShootingState.IDLE;
      if (mState == State.SHOOTING) {
        mShootingState = ShootingState.SHOOTING_COMPLETE;
      }
      if (mState == State.INTAKE) {
        mIntakeState = IntakeState.IDLE;
      }
    }

    if (mOperatorInterface.isBarf()) {
      if (mIntakeState == IntakeState.STORAGE_EJECT) {
        mIntakeState = IntakeState.IDLE;
      } else {
        mIntakeState = IntakeState.STORAGE_EJECT;
        mBarfTimer.reset();
        mBarfTimer.start();
      }
    }

    // TODO: REMOVE THIS IT SHOULDNT BE HERE
    // check if we are shooting
    // TODO: remove this and only allow shooting if you have at least 1 ball
    checkTransitionToShooting();

    updateSmartDashboard();

    if (mOperatorInterface.getSpinUp()) {
      sIsSpinningUp = !sIsSpinningUp;
    }

    // spin up shooter if commanded
    if (sIsSpinningUp) {
      mShooter.start();
    } else if (mState != State.SHOOTING) {
      mShooter.stop();
    }

    // Shooter velocity trim
    if (mShooterVelocityTrimDown.update(mOperatorInterface.getShooterVelocityTrimDown())) {
      // mShooter.decreaseVelocity();
    } else if (mShooterVelocityTrimUp.update(mOperatorInterface.getShooterVelocityTrimUp())) {
      // mShooter.increaseVelocity();
    } else if (mOperatorInterface.getResetVelocityTrim()) {
      mShooter.resetVelocity();
    }
    */
  }

  private void executeRobotStateMachine() {
    switch (mState) {
      case IDLE:
        SmartDashboard.putString("RobotState", mState.name());
        SmartDashboard.putString("IntakeState", mIntakeState.name());
        SmartDashboard.putString("ShootingState", mShootingState.name());
        SmartDashboard.putString("ClimbingState", mClimbingState.name());
        mIntake.stop();
        mStorage.stop();
        mShooter.stop();
        mClimber.stop();
        checkTransitionToClimbing();
        break;
      case INTAKE:
        executeIntakeStateMachine();
        checkTransitionToClimbing();
        break;
      case SHOOTING:
        executeShootingStateMachine();
        checkTransitionToClimbing();
        break;
      case CLIMBING:
        executeClimbingStateMachine();
        break;
      default:
        mRobotLogger.error("Invalid Robot State");
        break;
    }
  }

  private void executeIntakeStateMachine() {
    switch (mIntakeState) {
        // TODO: Make this not a transitionary state
      case IDLE:
        mIntake.stop();
        mStorage.stop();
        mShooter.stop();
        mIntakeState = IntakeState.READY_TO_INTAKE;
        break;
      case READY_TO_INTAKE:

        // If the operator issues the intake command, start intake
        if (mOperatorInterface.startIntake()) {
          mIntake.resetOvercurrentCooldown();
          mIntakeState = IntakeState.INTAKE;
        }
        break;
        // we wait until the garage door sensor is clear before moving to real intake
      case INTAKE_WAITING:
        mIntake.start();
        if (!mStorage.isBallDetected()) {
          mIntakeState = IntakeState.INTAKE;
        }
        break;
      case INTAKE:
        // Check transition to shooting before we start intake of a new ball
        if (!checkTransitionToShooting()) {
          mIntake.start();

          // If a ball is detected, store it
          if (mStorage.isBallDetected()) {
            if (mStorage.getBallCount() == mStorage.getCapacity() + 1) {
              mIntakeState = IntakeState.STORAGE_COMPLETE;
            }
            mStorage.updateEncoderPosition();
            mIntakeState = IntakeState.STORE_BALL;
          }

          if (mOperatorInterface.startIntake()) {
            mIntakeState = IntakeState.IDLE;
          }
        }
        break;
      case STORE_BALL:
        mStorage.storeBall();
        mIntake.stop();

        // If the sensor indicates the ball is stored, complete ball storage
        if (mStorage.isBallStored()) {
          mIntakeState = IntakeState.STORAGE_COMPLETE;
        }

        break;
        // we just stored a ball
      case STORAGE_COMPLETE:
        mStorage.addBall();
        mStorage.stop();

        // If the storage is not full, intake another ball
        if (!mStorage.isFull()) {
          mIntakeState = IntakeState.INTAKE_WAITING;
        }

        // Check transition to shooting after storage of ball
        checkTransitionToShooting();

        mIntake.resetOvercurrentCooldown();
        break;
      case STORAGE_EJECT:
        mStorage.updateEncoderPosition();
        mIntake.barf(); // Ball Acqusition Reverse Functionality (BARF)
        mStorage.barf();
        mStorage.emptyBalls();

        if (mBarfTimer.get() >= BARF_TIMER_DURATION) {
          mIntakeState = IntakeState.IDLE;
        }
        break;
      default:
        mRobotLogger.error("Invalid Intake State");
        break;
    }
  }

  private boolean checkTransitionToShooting() {
    // result.HasResult ensures that our vision system sees a target
    if (mOperatorInterface.getShoot() /* && (!mStorage.isEmpty()) && result.HasResult*/) {
      mRobotLogger.log("Changing to shoot because our driver said so...");
      switch (mState) {

          /** Disables intake if transitioning from intake */
        case INTAKE:
          mIntake.stop();
          mStorage.stop();
          mIntakeState = IntakeState.IDLE;
          break;
        default:
          break;
      }
      mState = State.SHOOTING;

      /** Sets the shooting state to preparing if it's not already */
      if (mShootingState == ShootingState.IDLE) {
        mShootingState = ShootingState.PREPARE_TO_SHOOT;
      }
      return true;
    } else {
      // mRobotLogger.info("Could not shoot because " + (!mStorage.isEmpty()) + " " +
      // mOperatorInterface.getShoot());
      return false;
    }
  }

  private boolean checkTransitionToClimbing() {
    // TODO: Remove the check that climber is enabled
    if (mOperatorInterface.climbStart() && Constants.Climber.enabled) {
      mRobotLogger.log("Changing to climbing");

      /** Disables intake if transitioning from intake */
      switch (mState) {
        case INTAKE:
          mIntake.stop();
          mStorage.stop();
          mIntakeState = IntakeState.IDLE;
          break;
        default:
          break;
      }

      /** Sets the climbing state to idle if it's not already */
      mState = State.CLIMBING;

      mDrive.setClimbingSpeed(true);

      if (mClimbingState == ClimbingState.IDLE) {
        mClimbingState = ClimbingState.WAIT;
      }
      return true;
    } else {
      return false;
    }
  }

  private boolean checkEscapeClimbHold() {
    if (mOperatorInterface.climbStart()) {
      mState = State.INTAKE;

      mIntakeState = IntakeState.READY_TO_INTAKE;

      mDrive.setClimbingSpeed(false);

      return true;
    } else {
      return false;
    }
  }

  private void executeShootingStateMachine() {
    switch (mShootingState) {
      case IDLE:
        mRobotLogger.warn("Shooting state is idle");
        mShooter.stop();
        break;
      case PREPARE_TO_SHOOT:
        mStorage.stop();

        /* Starts roller */
        mShooter.start();
        sIsSpinningUp = false;

        // make robot pass though cooldown timer
        // should help between shots and solve low velocity issue
        if (mCurrentCooldown > 0) {
          mCurrentCooldown--;
           // skip the rest of this loop
        }
        
        /* If rollers are spun up, changes to next state */
        if (mShooter.isAtVelocity() /* TODO: && Target Acquired */) {
          System.out.println("We are at velocity.. shoot  pls");
          mShootingState = ShootingState.SHOOT_BALL;
          // reset cooldown timer
          mCurrentCooldown = ShotCooldown;
        }else{
          System.out.println("NOT AT VELOCITY");
        }

        if (mOperatorInterface.getShoot()) {
          System.out.println("Is this booming our shot?");
          mShootingState = ShootingState.SHOOTING_COMPLETE;
          mStorage.stop();
        }
        break;
      case SHOOT_BALL:
        mStorage.ejectBall();
        System.out.println("Shoot the ball");
        mShooter.start();

        // turn off shooting
        if (mOperatorInterface.getShoot()) {
          System.out.println("Turn off shooting");
          mShootingState = ShootingState.SHOOTING_COMPLETE;
          mStorage.stop();
        }

        /* If finished shooting, changes to next state*/
        if (mShooter.isBallFired()) {
          System.out.println("is ball fired");
          //mShootingState = ShootingState.SHOOT_BALL_COMPLETE;
        }
        break;
      case SHOOT_BALL_COMPLETE:
        /* Decrements storage */
        mStorage.removeBall();

        // We are gonna continue shooting until
        // /* Goes to complete if storage is empty, otherwise fires again */
        // if (mStorage.isEmpty()) {
        //   mShootingState = ShootingState.SHOOTING_COMPLETE;
        // } else {
        //   mShootingState = ShootingState.PREPARE_TO_SHOOT;
        // }

        // shooting is a toggle
        mShootingState = ShootingState.PREPARE_TO_SHOOT;
        break;
      case SHOOTING_COMPLETE:
        System.out.println("Shooting complete???");
        /* Stops the roller and returns to intake state */
        mShooter.stop();
        mShootingState = ShootingState.IDLE;
        mState = State.INTAKE;
        break;
      default:
        mRobotLogger.error("Invalid shooting state");
        break;
    }
  }

  private void executeClimbingStateMachine() {
    if (mClimbingState != ClimbingState.IDLE) {
      SmartDashboard.putNumber("Climber pos", mClimber.getEncoderPosition());
    }

    switch (mClimbingState) {
      case IDLE:
        mClimber.stop();
        // mRobotLogger.warn("Climbing state is idle");
        break;
      case WAIT:
        if (mOperatorInterface.climbUp()) {
          mClimbingState = ClimbingState.EXTENDING;
        }

        checkEscapeClimbHold();
        break;
      case EXTENDING:
        // TODO: Decide if climb and retract should be the same button
        /** Checks if the climb button has been hit again, signalling it to retract */
        if (mOperatorInterface.climbUp()) {
          mClimber.extend();
        } else {
          mClimbingState = ClimbingState.HOLD;
        }

        /** Checks the encoder position to see if it's done climbing */
        if (mClimber.isExtended()) {
          mClimbingState = ClimbingState.EXTENDING_COMPLETE;
        }
        break;
      case EXTENDING_COMPLETE:
        mClimber.stop();

        /** Checks if the climb button has been hit again, signalling it to retract */
        if (mOperatorInterface.climbDown()) {
          mClimbingState = ClimbingState.RETRACTING;
        }
        break;
      case HOLD:
        mClimber.stop();

        if (mOperatorInterface.climbUp()) {
          mClimbingState = ClimbingState.EXTENDING;
        }

        if (mOperatorInterface.climbDown()) {
          mClimbingState = ClimbingState.RETRACTING;
        }

        checkEscapeClimbHold();

        break;
      case RETRACTING:
        mClimber.retract();

        if (mOperatorInterface.climbDown()) {
          mClimber.retract();
        } else {
          mClimbingState = ClimbingState.HOLD;
        }

        /** Checks the encoder position to see if it's done retracting */
        if (mClimber.isRetracted()) {
          mClimbingState = ClimbingState.WAIT;
        }
        break;
      default:
        mRobotLogger.error("Invalid Climbing State");
        break;
    }
    System.out.println(mClimber.getEncoderPosition());
  }

  private static void readIsPracticeBot() {
    // why do we invert it? because
    isPracticeBot = !practiceInput.get();
  }

  public static boolean getIsPracticeBot() {
    return isPracticeBot;
  }

  /**
   * Returns the robot's gyro. It should be noted that this gyro object's reported heading will
   * often <bold>not</bold> reflect the actual heading of the robot. This is because it is reset at
   * the beginning of every autonomous turn segment in order to allow relative turning.
   *
   * @return the gyro
   */
  public static Gyro getGyro() {
    return sGyro;
  }

  public static boolean isAuto() {
    return mAuto;
  }

  public static boolean getSpinningUp() {
    return sIsSpinningUp;
  }

  public static void setSpinningUp(boolean value) {
    sIsSpinningUp = value;
  }
}
