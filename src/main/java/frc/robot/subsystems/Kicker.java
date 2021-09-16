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
import frc.robot.SpeedLookupTable;

public class Kicker {

    enum KickerMode {
        Jog, //test mode... jogging
        Wind, //winding up
        ReadyToKick, //wound up, ready to kick
        Rewind,
        Kick,  //currently kicking
        Idle  //doing nothing
    };
    private SpeedLookupTable mSpeedLookupTable = SpeedLookupTable.getInstance();
    private TimeOfFlight mLidar = new TimeOfFlight(Constants.Talons.Storage.lidarCanID);
    List<Jaguar> allJags = new ArrayList<Jaguar>();
    //TODO: Test for actual range
    final double lidarDetectionDistance = 48;
    final int totalTicksInKick = 4100; //normal rotation is around 2048, this is to ensure kick through
    //Two jags per PWM slot, 12 Jags and motors total
    int totalJags = 6;
    double jagSpeed = 0;
    int selectedMotor = 0;
    AtomicInteger mTicksPerRotation = new AtomicInteger(totalTicksInKick);
    AtomicInteger mRotations = new AtomicInteger(1);
    double mTargetSpeed = 0;
    double mTargetYardline = 20;
    Encoder revEncoder = new Encoder(0, 1);
    private boolean mWoundUp = false;
    private int resetPos;
    int debounceWound = 10;

    public KickerMode mCurrentMode = KickerMode.Idle;


    
    private static Kicker sInstance;

    public static synchronized Kicker getInstance() {
        if (sInstance == null) {
            sInstance = new Kicker();
        }
        return sInstance;
    }

    public Kicker(){
        System.out.println("Kicker Setup!");
        mCurrentMode = KickerMode.Idle;
        NetworkTableInstance mNetTable = NetworkTableInstance.getDefault();
        NetworkTable CurrentNetworkTable = mNetTable.getTable("MotorConfig");
        NetworkTableEntry tickersPerRotation = CurrentNetworkTable.getEntry("ticksPerRotation");
        NetworkTableEntry numRotations = CurrentNetworkTable.getEntry("NumberRotations");
        NetworkTableEntry TargetSpeed = CurrentNetworkTable.getEntry("TargetSpeed");

        //defaults
        tickersPerRotation.setDouble(2048);
        numRotations.setDouble(1);
        TargetSpeed.setDouble(1);

        System.out.println("SETUP LISTENERS!");
        CurrentNetworkTable.addEntryListener("ticksPerRotation", (table, key, entry, value, flags) -> {
            System.out.println("Ticks changed value: " + value.getValue());
            int val = (int) value.getDouble();
            mTicksPerRotation.set(val);
            System.out.println("changed to: " + val);
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        CurrentNetworkTable.addEntryListener("NumberRotations", (table, key, entry, value, flags) -> {
            System.out.println("Rotations changed value: " + value.getValue());
            int val = (int) value.getDouble();
            mRotations.set(val);
            System.out.println("changed to: " + val);
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        CurrentNetworkTable.addEntryListener("TargetSpeed", (table, key, entry, value, flags) -> {
            System.out.println("targetSpeed changed value: " + value.getValue());
            mTargetSpeed = value.getDouble();
            System.out.println("changed to: " + mTargetSpeed);
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

    //constant update loop
    public void updateLoop(){
        if(mCurrentMode == KickerMode.Idle){
            //Do Nothing
            stop();
        }else if(mCurrentMode == KickerMode.Wind){
            //Winding
            windKicker();
        }else if(mCurrentMode == KickerMode.ReadyToKick){
            //Ready to Kink...wait
            stop();
        }else if(mCurrentMode == KickerMode.Kick){
            //Kicking
            performKick();
        }else if(mCurrentMode == KickerMode.Jog){
            
            //Test Mode.. Manually jogging
        }
    }



    // wind up 
    // allow double wind
    public void tryWind(){
        if(
            mCurrentMode == KickerMode.Idle ||
            mCurrentMode == KickerMode.ReadyToKick
        
        ){
            debounceWound = 30;
            System.out.println("Debounce Wound: " + debounceWound);
            mCurrentMode = KickerMode.Wind;
        }else{
            System.out.println("Not Ready to Wind");
        }
    }

    //Kick Mode if in wind
    public void tryKick(){
        if(mCurrentMode == KickerMode.ReadyToKick){
            //ready to kick, go to kick!
            mCurrentMode = KickerMode.Kick;
            zeroTicks();
        }else{
            System.out.println("Not Ready to Kick!");
        }
    }

    //performs the actual kick, called from update loop
    private void performKick(){
        int currentPos = Math.abs(getTicks());
        double currentRate = revEncoder.getRate();
        double distancePerPulse = revEncoder.getDistancePerPulse();
        double velocity = currentRate * distancePerPulse;
        int encoderPos = mTicksPerRotation.intValue() * mRotations.intValue();
        
        //lookup speed 
        jagSpeed = mSpeedLookupTable.getSpeedFromDistance(mTargetYardline);
        jagSpeed -= mTargetSpeed;

        //pull within range
        if(jagSpeed > 1){
            jagSpeed = 1;
        }
        if(jagSpeed < -1){
            jagSpeed = -1;
        }

        if (Math.abs(currentPos - resetPos) < encoderPos){
            //Continue to Kick
            updateSpeed();
        }
        else {
            //Kick Complete, back to idle
            mCurrentMode = KickerMode.Idle;
        }
    }

    public double encoderRate(){
        return revEncoder.getRate();
    }

    //windKicker
    private void windKicker(){
        if(mLidar.getRange() > lidarDetectionDistance
        ){
            System.out.println("KEEP WIND");
            //continue winding up
            jagSpeed = -.1;
            updateSpeed();
        }else if (debounceWound <= 0){
            System.out.println("STOP WINDING");
            //stop, wound up, ready to kick
            zeroTicks(); 
            mCurrentMode = KickerMode.ReadyToKick;
        }
        debounceWound--;
    }

    public void kick2(){
        int currentPos = Math.abs(getTicks());
        double currentRate = revEncoder.getRate();
        double distancePerPulse = revEncoder.getDistancePerPulse();
        double velocity = currentRate * distancePerPulse;
        int encoderPos = mTicksPerRotation.intValue() * mRotations.intValue();
        jagSpeed = mTargetSpeed;
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
        if( getLidarRange() > lidarDetectionDistance
            && debounceWound <= 0
        ){
            //continue winding up
            jagSpeed = -.1;
            mWoundUp = false;
        }else{
            //stop!
            mWoundUp = true;
            zeroTicks(); 
        }
        debounceWound--;
        updateSpeed();
    }

    public double getLidarRange(){
        return mLidar.getRange();

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

    public void setIdle(){
        mCurrentMode = KickerMode.Idle;
    }

    public void setJog(){
        mCurrentMode = KickerMode.Jog;
    }

    public void incrimentYardline(){
        mTargetYardline += 5;
        if(mTargetYardline > 100){
            mTargetYardline = 100;
        }
    }

    public void decrementYardLine(){
        mTargetYardline -= 5;
        if(mTargetYardline < 0){
            mTargetYardline = 0;
        }
    }

    public void incrimentSpeed(){
        mTargetSpeed += .05;
        if(mTargetSpeed >= 1){
            mTargetSpeed = 1;
        }
    }

    public void decrementSpeed(){
        mTargetSpeed -= .05;
        if(mTargetSpeed < 0){
            mTargetSpeed = 0;
        }
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
        SmartDashboard.putNumber("Target Speed", mTargetSpeed);
        SmartDashboard.putNumber("Lidar Sensor Range", getLidarRange());
        SmartDashboard.putNumber("Target Yardage", mTargetYardline);
        SmartDashboard.putNumber("Selected Speed", mSpeedLookupTable.getSpeedFromDistance(mTargetYardline));       

    }
}