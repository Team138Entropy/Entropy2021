package frc.robot.subsystems;
import java.util.ArrayList;
import java.util.List;
import java.math.*;
import java.nio.channels.NetworkChannel;

import edu.wpi.first.wpilibj.Encoder;
import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.concurrent.atomic.AtomicInteger;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.EntryListenerFlags;

import edu.wpi.first.wpilibj.Jaguar;
import frc.robot.Constants;

public class Kicker {

    private TimeOfFlight mLidar = new TimeOfFlight(Constants.Talons.Storage.lidarCanID);
    List<Jaguar> allJags = new ArrayList<Jaguar>();
    //TODO: Test for actual range
    final int detectionDistancee = 150;
    final int totalTicksInKick = 2048;
    //Two jags per PWM slot, 12 Jags and motors total
    int totalJags = 6;
    double jagSpeed = 0;
    int selectedMotor = 0;
    AtomicInteger mTicksPerRotation = new AtomicInteger(2048);
    AtomicInteger mRotations = new AtomicInteger(1);
    AtomicInteger mTargetSpeed = new AtomicInteger(1);
    Encoder revEncoder = new Encoder(0, 1);
    private boolean mWoundUp = false;
    private int resetPos;


    
    private static Kicker sInstance;

    public static synchronized Kicker getInstance() {
        if (sInstance == null) {
            sInstance = new Kicker();
        }
        return sInstance;
    }

    public Kicker(){
        System.out.println("Kicker Setup!");
        NetworkTableInstance mNetTable = NetworkTableInstance.getDefault();
        NetworkTable CurrentNetworkTable = mNetTable.getTable("MotorConfig");
        NetworkTableEntry tickersPerRotation = CurrentNetworkTable.getEntry("ticksPerRotation");
        NetworkTableEntry numRotations = CurrentNetworkTable.getEntry("numRotations");
        System.out.println("SETUP LISTENERS!");
        CurrentNetworkTable.addEntryListener("ticksPerRotation", (table, key, entry, value, flags) -> {
            System.out.println("Ticks changed value: " + value.getValue());
            int val = (int) value.getDouble();
            mTicksPerRotation.set(val);
            System.out.println("changed to: " + val);
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        CurrentNetworkTable.addEntryListener("numRotations", (table, key, entry, value, flags) -> {
            System.out.println("Rotations changed value: " + value.getValue());
            int val = (int) value.getDouble();
            mRotations.set(val);
            System.out.println("changed to: " + val);
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        mNetTable.startClientTeam(138);
        mNetTable.startDSClient();


        for(int i = 0; i < totalJags; i++){
            try{
                allJags.add(new Jaguar(i)); 
            } catch (Exception e){
                System.out.println("Exception adding Jaguar: " + e.getLocalizedMessage());
            }
        }
    }

    public void kick2(){
        int currentPos = Math.abs(getTicks());
        double currentRate = revEncoder.getRate();
        double distancePerPulse = revEncoder.getDistancePerPulse();
        double velocity = currentRate * distancePerPulse;
        int encoderPos = mTicksPerRotation.intValue() * mRotations.intValue();
        jagSpeed = mTargetSpeed.intValue();
        if (Math.abs(currentPos - resetPos) < encoderPos){
            updateSpeed();
        }
        else {
            stop();
        }
 

    }

    public void jogUp(){
        jagSpeed += .05;
        if(jagSpeed > 1){
            jagSpeed = 1;
        }
        updateSpeed(); 
    }

    public void jogDown(){
        jagSpeed -= .05;
        if(jagSpeed < -1){
            jagSpeed = -1;
        }
        updateSpeed();        
    }

    public void fakejogUp(){
        jagSpeed += .1;
        if(jagSpeed > 1){
            jagSpeed = 1;
        }
        System.out.println("Jag speed: " + jagSpeed);
    }

    public void fakejogDown(){
        jagSpeed -= .1;
        if(jagSpeed < -1){
            jagSpeed = -1;
        }
        System.out.println("Jag speed: " + jagSpeed);
    }

    public void kickReset2(){
        System.out.println("Kick Reset!");
        zeroTicks();
        resetPos = Math.abs(getTicks());
    }

    //update speed on all motors
    private void updateSpeed(){
        for(int i = 0; i < totalJags; i++){
            allJags.get(i).set(jagSpeed);
        }

    }

    //stop all motors
    public void stop(){
        jagSpeed = 0;
        for(int i = 0; i < totalJags; i++){
            allJags.get(i).set(0);
        }
    }

    public void windup(){
        if(mLidar.getRange() > detectionDistancee){
            //continue winding up
            jagSpeed = -.1;
            mWoundUp = false;
        }else{
            //stop!
            mWoundUp = true;
            zeroTicks(); 
        }
        updateSpeed();
    }

    public void kick(){
        if(!mWoundUp) return; //no kicking unless wound up

        //May need to ramp this
        if(!kickComplete()){
            //still have more kicks to go!
            jagSpeed = 1;
        }else{
            //reached end
            jagSpeed = 0;
            mWoundUp = false;
        }
    
        updateSpeed();
        System.out.println("Ticks: " + getTicks());
    }

    //
    public boolean kickComplete(){
        return  !(getTicks() < totalTicksInKick);
    }

    public void stopKicker(){
        jagSpeed = 0;
        updateSpeed();
    }

    public int getTicks(){
        return(revEncoder.get());
    }

    public void zeroTicks(){
        revEncoder.reset();
    }

    // update the smartdashboard with relevant info
    public void updateSmartdashboard(){
        double currentRate = revEncoder.getRate();
        double distancePerPulse = revEncoder.getDistancePerPulse();
        double velocity = currentRate * distancePerPulse;
        SmartDashboard.putNumber("Kicker Velocity", velocity);
        SmartDashboard.putNumber("Jag Speed", jagSpeed);
        SmartDashboard.putNumber("Ticks", getTicks());
        SmartDashboard.putNumber("Target Ticks", mTicksPerRotation.intValue());
        SmartDashboard.putNumber("Target Rotations", mRotations.intValue());
        SmartDashboard.putNumber("Target Speed", mTargetSpeed.intValue());
        //Update Mode
        SmartDashboard.putString("Mode", "Test Jogging");
   
    }
}