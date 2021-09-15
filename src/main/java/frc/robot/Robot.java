package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI.OperatorInterface;
import frc.robot.auto.DriveSegment;
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

import java.io.FileWriter;
import java.io.IOException;
import java.lang.reflect.Array;
import java.net.InetAddress;
import java.sql.Timestamp;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.concurrent.TimeUnit;
import java.text.SimpleDateFormat;


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
  private LatchedBoolean mYardsUp = new LatchedBoolean();
  private LatchedBoolean mYardsDown = new LatchedBoolean();
  private LatchedBoolean mSpeedUp = new LatchedBoolean();
  private LatchedBoolean mSpeedDown = new LatchedBoolean();

  FileWriter mCSVFile;
  boolean FileOpened = false;


  
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

  NetworkTableEntry xEntry;
  NetworkTableEntry yEntry;

  @Override
  public void teleopInit() {
    mKicker.setIdle();
    mKicker.zeroTicks();

    //Create a New Output CSV
    try{
      //close previous file 
      if(FileOpened){
        try{
          mCSVFile.close();
        }catch(IOException exception){
          //issue closing file
        }
      }

      Timestamp timestamp = new Timestamp(System.currentTimeMillis());
      mCSVFile = new FileWriter("OutputData-" + timestamp + ".csv");
      mCSVFile.write("Milliseconds, Encoder Velocity, Encoder Ticks\n");  
      FileOpened = true;
    }catch (IOException exception){

    }

    ///remove 
     //when your program starts
     NetworkTableInstance inst = NetworkTableInstance.getDefault();

     //Get the table within that instance that contains the data. There can
     //be as many tables as you like and exist to make it easier to organize
     //your data. In this case, it's a table called datatable.
     NetworkTable table = inst.getTable("datatable");

     //Get the entries within that table that correspond to the X and Y values
     //for some operation in your program.
     xEntry = table.getEntry("X");
     yEntry = table.getEntry("Y");
  }



  @Override
  public void teleopPeriodic() {
    xEntry.setDouble(0);
    yEntry.setDouble(0);
    /*
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
    else{
      mKicker.stop();
    }
*/
   // System.out.println(mKicker.getLidarRange());


    
      // Start State Machine Based Logic
      boolean kickPressed = mOperatorInterface.jogUp();
      boolean windPressed = mOperatorInterface.jogDown();
      boolean speedDownPressed = mOperatorInterface.speedDown();
      boolean speedUpPressed = mOperatorInterface.speedUp();
      boolean yardageUp = mOperatorInterface.yardsUp();
      boolean yardageDown = mOperatorInterface.yardsDown();
      if(mJogDown.update(windPressed)){
        //attempt wind if mode allows
        //allows a double wind
        System.out.println("Wind Pressed");
        mKicker.tryWind();
      }else if(mJogUp.update(kickPressed)){
        System.out.println("Kick Pressed");
        //attempt kick if mode allows
        mKicker.tryKick();
      }

     //control yardage
     if(mYardsUp.update((yardageUp))){
       mKicker.incrimentYardline();
     }else if(mYardsDown.update(yardageDown)){
       mKicker.decrementYardLine();
     }

     //control custom speed
     if(mSpeedUp.update(speedUpPressed)){
       mKicker.incrimentSpeed();
     }else if(mSpeedDown.update(speedDownPressed)){
       mKicker.decrementSpeed();
     }

     mKicker.updateLoop();
     // End State Machine Based Logic
  
   //End Logic

    // if(kickPressed){
    //   mKicker.kick();
    // }else if(windPressed){
    //   mKicker.windup();
    // }else{
    //   mKicker.stopKicker();
    // }

    


   driveLoop();
   mKicker.updateSmartdashboard();
   updateDataFile();
  }

  public void updateDataFile(){
    if(FileOpened){
      try{
        mCSVFile.write(String.valueOf(System.currentTimeMillis()));
        mCSVFile.write(",");
        mCSVFile.write(String.valueOf(mKicker.encoderRate()));
        mCSVFile.write(",");
        mCSVFile.write(String.valueOf(mKicker.getTicks()));
        mCSVFile.write("\n");
        mCSVFile.flush();
      }catch(IOException e){
        
      }
    }
  }

  public void driveLoop(){
    boolean driveForwardFiveYards = mOperatorInterface.driveForwardFiveYards();
    boolean driveBackwardsFiveYards = mOperatorInterface.driveBackwardsFiveYards();
    double driveThrottle = mOperatorInterface.getDriveThrottle();
    double driveTurn = mOperatorInterface.getDriveTurn();

    if(driveForwardFiveYards || driveBackwardsFiveYards){
      //Auto Drive +/- 5 Yards
      //this must be held to allow the path to continue
      mDrive.driveDistance(15, 
        Constants.Auto.defaultCruiseVelocity,
         Constants.Auto.defaultAccel, driveForwardFiveYards);
    }else{
      //Normal Drive
      mDrive.setDrive(driveThrottle, driveTurn, false);
    }
    mDrive.UpdateSmartdashboard();
  }

  @Override
  public void testPeriodic() {
    boolean jogDownPressed = mOperatorInterface.jogDown();
    boolean jogUpPressed = mOperatorInterface.jogUp();
    boolean jogResetPressed = mOperatorInterface.jogReset();
    boolean speedDownPressed = mOperatorInterface.speedDown();
    boolean speedUpPressed = mOperatorInterface.speedUp();
    boolean yardageUp = mOperatorInterface.yardsUp();
    boolean yardageDown = mOperatorInterface.yardsDown();
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
    mKicker.setJog();
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
