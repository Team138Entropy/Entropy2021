package frc.robot.subsystems;
import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.wpilibj.Encoder;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.Jaguar;
import frc.robot.Constants;

public class Kicker {

    private TimeOfFlight mLidar = new TimeOfFlight(Constants.Talons.Storage.lidarCanID);
    List<Jaguar> allJags = new ArrayList<Jaguar>();
    //TODO: Test for actual range
    int detectionDistancee = 150;
    int totalTicksInKick = 5000;
    //Two jags per PWM slot, 12 Jags and motors total
    int totalJags = 6;
    double jagSpeed = 0;
    int selectedMotor = 0;
    Encoder revEncoder = new Encoder(6, 7);
    private boolean mWoundUp = false;

    
    private static Kicker sInstance;

    public static synchronized Kicker getInstance() {
        if (sInstance == null) {
            sInstance = new Kicker();
        }
        return sInstance;
    }

    public Kicker(){
        for(int i = 0; i < totalJags; i++){
            try{
                allJags.add(new Jaguar(i)); 
            } catch (Exception e){
            }
        }
    }

    public void jogUp(){
        jagSpeed += .1;
        if(jagSpeed > 1){
            jagSpeed = 1;
        }
        updateSpeed(); 
    }

    public void jogDown(){
        jagSpeed -= .1;
        if(jagSpeed < 0){
            jagSpeed =0;
        }
        updateSpeed();        
    }

    private void updateSpeed(){
        //If selectMotor is 6, all jugs run
        if(selectedMotor == totalJags){
            for(int i = 0; i < totalJags; i++){
                allJags.get(i).set(jagSpeed);
            }
        }
        else{
            allJags.get(selectedMotor).set(jagSpeed);
        }
    }

    public void stop(){
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

    public void selectMotorUp(){
        selectedMotor = selectedMotor + 1;
        if (selectedMotor > totalJags){
            selectedMotor = 0;
        }
        if(selectedMotor != 6){
            System.out.println("Motor selected: " + selectedMotor);
        }
        else{
            System.out.println("All motors");
        }
    }

    public void selectMotorDown(){
        selectedMotor = selectedMotor - 1;
        if (selectedMotor < 0){
            selectedMotor = totalJags;
        }
        if(selectedMotor != 6){
            System.out.println("Motor selected: " + selectedMotor);
        }
        else{
            System.out.println("All motors");
        }
    }
}