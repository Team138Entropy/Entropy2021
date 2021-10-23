package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import frc.robot.OurWPITalonSRX;
import frc.robot.Robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

class PIDRoller {

  private final int PID_LOOP_INDEX = 0;
  private final int TIMEOUT_MS = 10;

  //private final OurWPITalonSRX //mTalon;

  private final WPI_TalonFX FalconMotor1;
  private final WPI_TalonFX FalconMotor2;

  private int rollupLevel = -1;

  private double speedSetPoint = 0;
  private boolean atSetpointTarget = false;

  PIDRoller(int talonPort, int talon2Port, double p, double i, double d, double f) {
    FalconMotor1 = new WPI_TalonFX(talonPort);
    FalconMotor2 = new WPI_TalonFX(talon2Port);    
    //mTalon = new OurWPITalonSRX(talonPort);
    //mTalonSlave = new OurWPITalonSRX(talon2Port);   

    FalconMotor1.configFactoryDefault();
    FalconMotor2.configFactoryDefault();
    //mTalon.configFactoryDefault();
    //mTalonSlave.configFactoryDefault();

    // All of this was ripped from the 2019 elevator code
    FalconMotor1.configNominalOutputForward(0, TIMEOUT_MS);
    FalconMotor1.configNominalOutputReverse(0, TIMEOUT_MS);
    FalconMotor1.configPeakOutputForward(1, TIMEOUT_MS);
    FalconMotor1.configPeakOutputReverse(-1, TIMEOUT_MS);
    //mTalon.configNominalOutputForward(0, TIMEOUT_MS);
    //mTalon.configNominalOutputReverse(0, TIMEOUT_MS);
    //mTalon.configPeakOutputForward(1, TIMEOUT_MS);
    //mTalon.configPeakOutputReverse(-1, TIMEOUT_MS);

    FalconMotor1.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_100Ms, 50);
    FalconMotor1.configVelocityMeasurementWindow(64);
    FalconMotor1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
    FalconMotor1.setSensorPhase(!Robot.getIsPracticeBot());
    //mTalon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_100Ms, 50);
    //mTalon.configVelocityMeasurementWindow(64);
    //mTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    //mTalon.setSensorPhase(!Robot.getIsPracticeBot());


    //FalconMotor1.enableVoltageCompensation(true); // turn on/off feature
    //FalconMotor1.configAllowableClosedloopError(PID_LOOP_INDEX, 50, TIMEOUT_MS);
    //mTalon.enableVoltageCompensation(true); // turn on/off feature
    //mTalon.configAllowableClosedloopError(PID_LOOP_INDEX, 50, TIMEOUT_MS);


    /*
    FalconMotor1.config_kP(PID_LOOP_INDEX, p);
    FalconMotor1.config_kI(PID_LOOP_INDEX, i);
    FalconMotor1.config_kD(PID_LOOP_INDEX, d);
    FalconMotor1.config_kF(PID_LOOP_INDEX, f);
    */
   // FalconMotor1.configClosedloopRamp(15);
    //mTalon.config_kP(PID_LOOP_INDEX, p);
    //mTalon.config_kI(PID_LOOP_INDEX, i);
    //mTalon.config_kD(PID_LOOP_INDEX, d);
    //mTalon.config_kF(PID_LOOP_INDEX, f);

    FalconMotor1.setNeutralMode(NeutralMode.Coast);
    //mTalon.setNeutralMode(NeutralMode.Coast);

    FalconMotor1.config_IntegralZone(PID_LOOP_INDEX, 200, TIMEOUT_MS);
    //mTalon.config_IntegralZone(PID_LOOP_INDEX, 200, TIMEOUT_MS);

    FalconMotor1.setInverted(true);
    //if (!Robot.getIsPracticeBot()) //mTalon.setInverted(true);
    
    // Set to Slave Mode
    FalconMotor2.follow(FalconMotor1); 
    //mTalonSlave.follow(//mTalon);
  }

  public int getVelocity() {
    return FalconMotor1.getSelectedSensorVelocity();
    //return -mTalon.getSelectedSensorVelocity();
  }

  public double getCurrent() {
    return FalconMotor1.getStatorCurrent();
    //return mTalon.getStatorCurrent();
  }

  //set
  public void setOutput(double output){

  }

  void setPercentOutput(double output) {
    // //System.out.println(getVelocity() + " velocity at output " + output); 
    if(output == 0){
      FalconMotor1.set(ControlMode.PercentOutput, 0);
      speedSetPoint = 0;
      atSetpointTarget = false;
    }else{
      //System.out.println("SETTING PERC OUTPUT: " + output);

      /*
     if(getVelocity() < 3000){
       FalconMotor1.set(ControlMode.PercentOutput, .2);
     }else if(getVelocity()  < 6000){
       FalconMotor1.set(ControlMode.PercentOutput, .5);
     }
     else{
       FalconMotor1.set(ControlMode.PercentOutput, output);
     }
     */
    if(speedSetPoint < output){
      speedSetPoint += .01;
    }
    if(speedSetPoint >= 1){
      atSetpointTarget = true;
    }
    if(atSetpointTarget){
      FalconMotor1.set(ControlMode.PercentOutput, output);
    }else{
      FalconMotor1.set(ControlMode.PercentOutput, speedSetPoint);
    }

     // //System.out.println("setPercentOutput: " + output;)
     // FalconMotor1.set(ControlMode.PercentOutput, speedSetPoint);
    }
    //mTalon.set(ControlMode.PercentOutput, -output);
  }

  void setSpeed(int posPer100Ms) {
    if (posPer100Ms == 0) {
      FalconMotor1.set(ControlMode.PercentOutput, 0);
      //mTalon.set(ControlMode.PercentOutput, 0);
    } else {
      FalconMotor1.set(ControlMode.Velocity, -posPer100Ms);
      //mTalon.set(ControlMode.Velocity, -posPer100Ms);
    }
  }
}
