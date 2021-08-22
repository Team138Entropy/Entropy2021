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
    //Two jags per PWM slot, 12 Jags and motors total
    int totalJags = 6;
    int jagSpeed = 0;
    int selectedMotor = 0;
    Encoder revEncoder = new Encoder(6, 7);

    
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
        //If selectMotor is 6, all jags run
        if(selectedMotor == totalJags){
            for(int i = 0; i < totalJags; i++){
                allJags.get(i).set(jagSpeed + .1);
            }
        }
        else{
            allJags.get(selectedMotor).set(jagSpeed + .1);
        }
    }

    public void jogDown(){
        //If selectMotor is 6, all jags run
        if(selectedMotor == totalJags){
            for(int i = 0; i < totalJags; i++){
                allJags.get(i).set(jagSpeed - .1);
            }
        }
        else{
            allJags.get(selectedMotor).set(jagSpeed - .1);
        }
        
    }

    public void stop(){
        for(int i = 0; i < totalJags; i++){
            allJags.get(i).set(0);
        }
    }

    public void windup(){
        while(mLidar.getRange() > detectionDistancee){
            for(int i = 0; i < totalJags; i++){
                allJags.get(i).set(.1);
            }
        }
    }

    public void kick(){
        //May need to ramp this
        if(selectedMotor == totalJags){
            for(int i = 0; i < totalJags; i++){
                allJags.get(i).set(1);
            }
        }
        else{
            allJags.get(selectedMotor).set(1);
        }
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