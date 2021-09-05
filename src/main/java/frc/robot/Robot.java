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
 
  // Controller Reference
  private final OperatorInterface mOperatorInterface = OperatorInterface.getInstance();
  private final Kicker mKicker = Kicker.getInstance();
  private final Drive mDrive = Drive.getInstance();

  //kicker latched booleans
  private LatchedBoolean mJogUp = new LatchedBoolean();
  private LatchedBoolean mJogDown = new LatchedBoolean();
  private LatchedBoolean mJogReset = new LatchedBoolean();
  private LatchedBoolean mJogFire = new LatchedBoolean();

  
  /**
   * The robot's gyro. Don't use this for absolute measurements. See {@link #getGyro()} for more
   * details.
   *
   * @see #getGyro()
   */
  private static ADXRS450_Gyro sGyro = new ADXRS450_Gyro();

  

  //private final NykoController OperatorController;

  @Override

  public void robotInit() {

  }

  @Override
  public void robotPeriodic() {
  }



  @Override
  public void autonomousInit() {

  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopInit() {
    mKicker.zeroTicks();
  }

  @Override
  public void teleopPeriodic() {
    boolean firePressed = mOperatorInterface.fireTrigger();
    boolean kickPressed = mOperatorInterface.jogUp();
    boolean windPressed = mOperatorInterface.jogDown();
    boolean resetPressed = mOperatorInterface.jogReset();

    if(mJogUp.update(kickPressed)){
      //jog up
      mKicker.fakejogUp();
    }else if(mJogDown.update(windPressed)){
      //jog down
      mKicker.fakejogDown();
    }
    else if (firePressed){
      mKicker.kick2();
    }
    else if (mJogReset.update(resetPressed)){
      mKicker.kickReset2();
    }

    //System.out.println("Ticks: " + mKicker.getTicks());



    // if(kickPressed){
    //   mKicker.kick();
    // }else if(windPressed){
    //   mKicker.windup();
    // }else{
    //   mKicker.stopKicker();
    // }

   mKicker.updateSmartdashboard();
   //driveLoop();
  }

  public void driveLoop(){
    double driveThrottle = mOperatorInterface.getDriveThrottle();
    double driveTurn = mOperatorInterface.getDriveTurn();
    mDrive.setDrive(driveThrottle, driveTurn, false);
  }

  @Override
  public void testPeriodic() {
    boolean jogDownPressed = mOperatorInterface.jogDown();
    boolean jogUpPressed = mOperatorInterface.jogUp();
    boolean jogResetPressed = mOperatorInterface.jogReset();
    if(mJogUp.update(jogUpPressed)){
      //jog up
      mKicker.jogUp();
    }else if(mJogDown.update(jogDownPressed)){
      //jog down
      mKicker.jogDown();
    }else if(mJogReset.update(jogResetPressed)){
      //jog stop
      mKicker.stop();
    }
    mKicker.updateSmartdashboard();
  }

  @Override
  public void testInit() {
    mKicker.zeroTicks();

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




  @Override
  public void disabledInit() {
    
  }

  @Override
  public void disabledPeriodic() {}

  



  private static void readIsPracticeBot() {
    // why do we invert it? because
  }

  public static boolean getIsPracticeBot() {
    return false;
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
    return false;
  }

  public static boolean getSpinningUp() {
    return false;
  }

  public static void setSpinningUp(boolean value) {
  }
}
