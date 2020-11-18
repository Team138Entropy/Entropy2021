package frc.robot;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

/**
 * This class is a thin wrapper around the CANTalon that reduces CAN bus / CPU overhead by skipping
 * duplicate set commands. (By default the Talon flushes the Tx buffer on every set call).
 */
public class LazyWPITalonSRX {
  protected double mLastSet = Double.NaN;
  protected ControlMode mLastControlMode = null;
  private WPI_TalonSRX thisTalon;

  public LazyWPITalonSRX(int deviceNumber) {
    if (Robot.isReal()) {
      thisTalon = new WPI_TalonSRX(deviceNumber);
    }
  }

  public double getLastSet() {
    return mLastSet;
  }

  public void set(ControlMode mode, double value) {
    if (Robot.isReal() && (value != mLastSet || mode != mLastControlMode)) {
      mLastSet = value;
      mLastControlMode = mode;
      thisTalon.set(mode, value);
    }
  }

  public void set(double speed) {
    thisTalon.set(speed);
  }

  public void pidWrite(double output) {
    thisTalon.pidWrite(output);
  }

  public double get() {
    return thisTalon.get();
  }

  public void set(ControlMode mode, double value) {
    thisTalon.set(mode, value);
  }

  public void set(ControlMode mode, double demand0, double demand1) {
    thisTalon.set(mode, demand0, demand1);
  }

  public void set(ControlMode mode, double demand0, DemandType demand1Type, double demand1) {
    thisTalon.set(mode, demand0, demand1Type, demand1);
  }

  public void setVoltage(double outputVolts) {
    thisTalon.setVoltage(outputVolts);
  }

  public void setInverted(boolean isInverted) {
    thisTalon.setInverted(isInverted);
  }

  public boolean getInverted() {
    return thisTalon.getInverted();
  }

  public void disable() {
    thisTalon.disable();
  }

  public void stopMotor() {
    thisTalon.stopMotor();
  }

  public void free() {
    thisTalon.free();
  }

  public final synchronized String getName() {
    return thisTalon.getName();
  }

  public final synchronized void setName(String name) {
    thisTalon.setName(name);
  }

  public final void setName(String moduleType, int channel) {
    thisTalon.setName(moduleType, channel);
  }

  public final void setName(String moduleType, int moduleNumber, int channel) {
    thisTalon.setName(moduleType, moduleNumber, channel);
  }

  public final synchronized String getSubsystem() {
    return thisTalon.getSubsystem();
  }

  public final synchronized void setSubsystem(String subsystem) {
    thisTalon.setSubsystem(subsystem);
  }

  public final void addChild(Object child) {
    thisTalon.addChild(child);
  }

  public void initSendable(SendableBuilder builder) {
    thisTalon.initSendable(builder);
  }

  public String getDescription() {
    return thisTalon.getDescription();
  }

  public void feed() {
    thisTalon.feed();
  }

  public void setExpiration(double expirationTime) {
    thisTalon.setExpiration(expirationTime);
  }

  public double getExpiration() {
    return thisTalon.getExpiration();
  }

  public boolean isAlive() {
    return thisTalon.isAlive();
  }

  public void setSafetyEnabled(boolean enabled) {
    thisTalon.setSafetyEnabled(enabled);
  }

  public boolean isSafetyEnabled() {
    return thisTalon.isSafetyEnabled();
  }

  public void set(TalonSRXControlMode mode, double value) {
    thisTalon.set(mode, value);
  }

  public void set(
      TalonSRXControlMode mode, double demand0, DemandType demand1Type, double demand1) {
    thisTalon.set(mode, demand0, demand1Type, demand1);
  }

  public SensorCollection getSensorCollection() {
    return thisTalon.getSensorCollection();
  }

  public ErrorCode configSelectedFeedbackSensor(
      TalonSRXFeedbackDevice feedbackDevice, int pidIdx, int timeoutMs) {
    return thisTalon.configSelectedFeedbackSensor(feedbackDevice, pidIdx, timeoutMs);
  }

  public ErrorCode configSupplyCurrentLimit(
      SupplyCurrentLimitConfiguration currLimitConfigs, int timeoutMs) {
    return thisTalon.configSupplyCurrentLimit(currLimitConfigs, timeoutMs);
  }

  public ErrorCode configSupplyCurrentLimit(SupplyCurrentLimitConfiguration currLimitConfigs) {
    return thisTalon.configSupplyCurrentLimit(currLimitConfigs);
  }

  public ErrorCode configPeakCurrentLimit(int amps, int timeoutMs) {
    return thisTalon.configPeakCurrentLimit(amps, timeoutMs);
  }

  public ErrorCode configPeakCurrentLimit(int amps) {
    return thisTalon.configPeakCurrentLimit(amps);
  }

  public ErrorCode configPeakCurrentDuration(int milliseconds, int timeoutMs) {
    return thisTalon.configPeakCurrentDuration(milliseconds, timeoutMs);
  }

  public ErrorCode configPeakCurrentDuration(int milliseconds) {
    return thisTalon.configPeakCurrentDuration(milliseconds);
  }

  public ErrorCode configContinuousCurrentLimit(int amps, int timeoutMs) {
    return thisTalon.configContinuousCurrentLimit(amps, timeoutMs);
  }

  public ErrorCode configContinuousCurrentLimit(int amps) {
    return thisTalon.configContinuousCurrentLimit(amps);
  }

  public void enableCurrentLimit(boolean enable) {
    thisTalon.enableCurrentLimit(enable);
  }

  public void getPIDConfigs(TalonSRXPIDSetConfiguration pid, int pidIdx, int timeoutMs) {
    thisTalon.getPIDConfigs(pid, pidIdx, timeoutMs);
  }

  public void getPIDConfigs(TalonSRXPIDSetConfiguration pid) {
    thisTalon.getPIDConfigs(pid);
  }

  public ErrorCode configAllSettings(TalonSRXConfiguration allConfigs, int timeoutMs) {
    return thisTalon.configAllSettings(allConfigs, timeoutMs);
  }

  public ErrorCode configAllSettings(TalonSRXConfiguration allConfigs) {
    return thisTalon.configAllSettings(allConfigs);
  }

  public void getAllConfigs(TalonSRXConfiguration allConfigs, int timeoutMs) {
    thisTalon.getAllConfigs(allConfigs, timeoutMs);
  }

  public void getAllConfigs(TalonSRXConfiguration allConfigs) {
    thisTalon.getAllConfigs(allConfigs);
  }

  public ErrorCode setStatusFramePeriod(StatusFrameEnhanced frame, int periodMs, int timeoutMs) {
    return thisTalon.setStatusFramePeriod(frame, periodMs, timeoutMs);
  }

  public ErrorCode setStatusFramePeriod(StatusFrameEnhanced frame, int periodMs) {
    return thisTalon.setStatusFramePeriod(frame, periodMs);
  }

  public int getStatusFramePeriod(StatusFrameEnhanced frame, int timeoutMs) {
    return thisTalon.getStatusFramePeriod(frame, timeoutMs);
  }

  public int getStatusFramePeriod(StatusFrameEnhanced frame) {
    return thisTalon.getStatusFramePeriod(frame);
  }

  public double getOutputCurrent() {
    return thisTalon.getOutputCurrent();
  }

  public double getStatorCurrent() {
    return thisTalon.getStatorCurrent();
  }

  public double getSupplyCurrent() {
    return thisTalon.getSupplyCurrent();
  }

  public ErrorCode configVelocityMeasurementPeriod(VelocityMeasPeriod period, int timeoutMs) {
    return thisTalon.configVelocityMeasurementPeriod(period, timeoutMs);
  }

  public ErrorCode configVelocityMeasurementPeriod(VelocityMeasPeriod period) {
    return thisTalon.configVelocityMeasurementPeriod(period);
  }

  public ErrorCode configVelocityMeasurementWindow(int windowSize, int timeoutMs) {
    return thisTalon.configVelocityMeasurementWindow(windowSize, timeoutMs);
  }

  public ErrorCode configVelocityMeasurementWindow(int windowSize) {
    return thisTalon.configVelocityMeasurementWindow(windowSize);
  }

  public ErrorCode configForwardLimitSwitchSource(
      LimitSwitchSource type, LimitSwitchNormal normalOpenOrClose, int timeoutMs) {
    return thisTalon.configForwardLimitSwitchSource(type, normalOpenOrClose, timeoutMs);
  }

  public ErrorCode configForwardLimitSwitchSource(
      LimitSwitchSource type, LimitSwitchNormal normalOpenOrClose) {
    return thisTalon.configForwardLimitSwitchSource(type, normalOpenOrClose);
  }

  public ErrorCode configReverseLimitSwitchSource(
      LimitSwitchSource type, LimitSwitchNormal normalOpenOrClose, int timeoutMs) {
    return thisTalon.configReverseLimitSwitchSource(type, normalOpenOrClose, timeoutMs);
  }

  public ErrorCode configReverseLimitSwitchSource(
      LimitSwitchSource type, LimitSwitchNormal normalOpenOrClose) {
    return thisTalon.configReverseLimitSwitchSource(type, normalOpenOrClose);
  }

  public int isFwdLimitSwitchClosed() {
    return thisTalon.isFwdLimitSwitchClosed();
  }

  public int isRevLimitSwitchClosed() {
    return thisTalon.isRevLimitSwitchClosed();
  }

  public ErrorCode DestroyObject() {
    return thisTalon.DestroyObject();
  }

  public long getHandle() {
    return thisTalon.getHandle();
  }

  public int getDeviceID() {
    return thisTalon.getDeviceID();
  }

  public void neutralOutput() {
    thisTalon.neutralOutput();
  }

  public void setNeutralMode(NeutralMode neutralMode) {
    thisTalon.setNeutralMode(neutralMode);
  }

  public void enableHeadingHold(boolean enable) {
    thisTalon.enableHeadingHold(enable);
  }

  public void selectDemandType(boolean value) {
    thisTalon.selectDemandType(value);
  }

  public void setSensorPhase(boolean PhaseSensor) {
    thisTalon.setSensorPhase(PhaseSensor);
  }

  public void setInverted(InvertType invertType) {
    thisTalon.setInverted(invertType);
  }

  public ErrorCode configFactoryDefault(int timeoutMs) {
    return thisTalon.configFactoryDefault(timeoutMs);
  }

  public ErrorCode configFactoryDefault() {
    return thisTalon.configFactoryDefault();
  }

  public ErrorCode configOpenloopRamp(double secondsFromNeutralToFull, int timeoutMs) {
    return thisTalon.configOpenloopRamp(secondsFromNeutralToFull, timeoutMs);
  }

  public ErrorCode configOpenloopRamp(double secondsFromNeutralToFull) {
    return thisTalon.configOpenloopRamp(secondsFromNeutralToFull);
  }

  public ErrorCode configClosedloopRamp(double secondsFromNeutralToFull, int timeoutMs) {
    return thisTalon.configClosedloopRamp(secondsFromNeutralToFull, timeoutMs);
  }

  public ErrorCode configClosedloopRamp(double secondsFromNeutralToFull) {
    return thisTalon.configClosedloopRamp(secondsFromNeutralToFull);
  }

  public ErrorCode configPeakOutputForward(double percentOut, int timeoutMs) {
    return thisTalon.configPeakOutputForward(percentOut, timeoutMs);
  }

  public ErrorCode configPeakOutputForward(double percentOut) {
    return thisTalon.configPeakOutputForward(percentOut);
  }

  public ErrorCode configPeakOutputReverse(double percentOut, int timeoutMs) {
    return thisTalon.configPeakOutputReverse(percentOut, timeoutMs);
  }

  public ErrorCode configPeakOutputReverse(double percentOut) {
    return thisTalon.configPeakOutputReverse(percentOut);
  }

  public ErrorCode configNominalOutputForward(double percentOut, int timeoutMs) {
    return thisTalon.configNominalOutputForward(percentOut, timeoutMs);
  }

  public ErrorCode configNominalOutputForward(double percentOut) {
    return thisTalon.configNominalOutputForward(percentOut);
  }

  public ErrorCode configNominalOutputReverse(double percentOut, int timeoutMs) {
    return thisTalon.configNominalOutputReverse(percentOut, timeoutMs);
  }

  public ErrorCode configNominalOutputReverse(double percentOut) {
    return thisTalon.configNominalOutputReverse(percentOut);
  }

  public ErrorCode configNeutralDeadband(double percentDeadband, int timeoutMs) {
    return thisTalon.configNeutralDeadband(percentDeadband, timeoutMs);
  }

  public ErrorCode configNeutralDeadband(double percentDeadband) {
    return thisTalon.configNeutralDeadband(percentDeadband);
  }

  public ErrorCode configVoltageCompSaturation(double voltage, int timeoutMs) {
    return thisTalon.configVoltageCompSaturation(voltage, timeoutMs);
  }

  public ErrorCode configVoltageCompSaturation(double voltage) {
    return thisTalon.configVoltageCompSaturation(voltage);
  }

  public ErrorCode configVoltageMeasurementFilter(int filterWindowSamples, int timeoutMs) {
    return thisTalon.configVoltageMeasurementFilter(filterWindowSamples, timeoutMs);
  }

  public ErrorCode configVoltageMeasurementFilter(int filterWindowSamples) {
    return thisTalon.configVoltageMeasurementFilter(filterWindowSamples);
  }

  public void enableVoltageCompensation(boolean enable) {
    thisTalon.enableVoltageCompensation(enable);
  }

  public boolean isVoltageCompensationEnabled() {
    return thisTalon.isVoltageCompensationEnabled();
  }

  public double getBusVoltage() {
    return thisTalon.getBusVoltage();
  }

  public double getMotorOutputPercent() {
    return thisTalon.getMotorOutputPercent();
  }

  public double getMotorOutputVoltage() {
    return thisTalon.getMotorOutputVoltage();
  }

  public double getTemperature() {
    return thisTalon.getTemperature();
  }

  public ErrorCode configSelectedFeedbackSensor(
      RemoteFeedbackDevice feedbackDevice, int pidIdx, int timeoutMs) {
    return thisTalon.configSelectedFeedbackSensor(feedbackDevice, pidIdx, timeoutMs);
  }

  public ErrorCode configSelectedFeedbackSensor(RemoteFeedbackDevice feedbackDevice) {
    return thisTalon.configSelectedFeedbackSensor(feedbackDevice);
  }

  public ErrorCode configSelectedFeedbackSensor(
      FeedbackDevice feedbackDevice, int pidIdx, int timeoutMs) {
    return thisTalon.configSelectedFeedbackSensor(feedbackDevice, pidIdx, timeoutMs);
  }

  public ErrorCode configSelectedFeedbackSensor(FeedbackDevice feedbackDevice) {
    return thisTalon.configSelectedFeedbackSensor(feedbackDevice);
  }

  public ErrorCode configSelectedFeedbackCoefficient(
      double coefficient, int pidIdx, int timeoutMs) {
    return thisTalon.configSelectedFeedbackCoefficient(coefficient, pidIdx, timeoutMs);
  }

  public ErrorCode configSelectedFeedbackCoefficient(double coefficient) {
    return thisTalon.configSelectedFeedbackCoefficient(coefficient);
  }

  public ErrorCode configRemoteFeedbackFilter(
      int deviceID, RemoteSensorSource remoteSensorSource, int remoteOrdinal, int timeoutMs) {
    return thisTalon.configRemoteFeedbackFilter(
        deviceID, remoteSensorSource, remoteOrdinal, timeoutMs);
  }

  public ErrorCode configRemoteFeedbackFilter(
      int deviceID, RemoteSensorSource remoteSensorSource, int remoteOrdinal) {
    return thisTalon.configRemoteFeedbackFilter(deviceID, remoteSensorSource, remoteOrdinal);
  }

  public ErrorCode configRemoteFeedbackFilter(
      CANCoder canCoderRef, int remoteOrdinal, int timeoutMs) {
    return thisTalon.configRemoteFeedbackFilter(canCoderRef, remoteOrdinal, timeoutMs);
  }

  public ErrorCode configRemoteFeedbackFilter(CANCoder canCoderRef, int remoteOrdinal) {
    return thisTalon.configRemoteFeedbackFilter(canCoderRef, remoteOrdinal);
  }

  public ErrorCode configSensorTerm(
      SensorTerm sensorTerm, FeedbackDevice feedbackDevice, int timeoutMs) {
    return thisTalon.configSensorTerm(sensorTerm, feedbackDevice, timeoutMs);
  }

  public ErrorCode configSensorTerm(SensorTerm sensorTerm, FeedbackDevice feedbackDevice) {
    return thisTalon.configSensorTerm(sensorTerm, feedbackDevice);
  }

  public ErrorCode configSensorTerm(
      SensorTerm sensorTerm, RemoteFeedbackDevice feedbackDevice, int timeoutMs) {
    return thisTalon.configSensorTerm(sensorTerm, feedbackDevice, timeoutMs);
  }

  public ErrorCode configSensorTerm(SensorTerm sensorTerm, RemoteFeedbackDevice feedbackDevice) {
    return thisTalon.configSensorTerm(sensorTerm, feedbackDevice);
  }

  public int getSelectedSensorPosition(int pidIdx) {
    return thisTalon.getSelectedSensorPosition(pidIdx);
  }

  public int getSelectedSensorPosition() {
    return thisTalon.getSelectedSensorPosition();
  }

  public int getSelectedSensorVelocity(int pidIdx) {
    return thisTalon.getSelectedSensorVelocity(pidIdx);
  }

  public int getSelectedSensorVelocity() {
    return thisTalon.getSelectedSensorVelocity();
  }

  public ErrorCode setSelectedSensorPosition(int sensorPos, int pidIdx, int timeoutMs) {
    return thisTalon.setSelectedSensorPosition(sensorPos, pidIdx, timeoutMs);
  }

  public ErrorCode setSelectedSensorPosition(int sensorPos) {
    return thisTalon.setSelectedSensorPosition(sensorPos);
  }

  public ErrorCode setControlFramePeriod(ControlFrame frame, int periodMs) {
    return thisTalon.setControlFramePeriod(frame, periodMs);
  }

  public ErrorCode setControlFramePeriod(int frame, int periodMs) {
    return thisTalon.setControlFramePeriod(frame, periodMs);
  }

  public ErrorCode setStatusFramePeriod(int frameValue, int periodMs, int timeoutMs) {
    return thisTalon.setStatusFramePeriod(frameValue, periodMs, timeoutMs);
  }

  public ErrorCode setStatusFramePeriod(int frameValue, int periodMs) {
    return thisTalon.setStatusFramePeriod(frameValue, periodMs);
  }

  public ErrorCode setStatusFramePeriod(StatusFrame frame, int periodMs, int timeoutMs) {
    return thisTalon.setStatusFramePeriod(frame, periodMs, timeoutMs);
  }

  public ErrorCode setStatusFramePeriod(StatusFrame frame, int periodMs) {
    return thisTalon.setStatusFramePeriod(frame, periodMs);
  }

  public int getStatusFramePeriod(int frame, int timeoutMs) {
    return thisTalon.getStatusFramePeriod(frame, timeoutMs);
  }

  public int getStatusFramePeriod(int frame) {
    return thisTalon.getStatusFramePeriod(frame);
  }

  public int getStatusFramePeriod(StatusFrame frame, int timeoutMs) {
    return thisTalon.getStatusFramePeriod(frame, timeoutMs);
  }

  public int getStatusFramePeriod(StatusFrame frame) {
    return thisTalon.getStatusFramePeriod(frame);
  }

  public ErrorCode configForwardLimitSwitchSource(
      RemoteLimitSwitchSource type,
      LimitSwitchNormal normalOpenOrClose,
      int deviceID,
      int timeoutMs) {
    return thisTalon.configForwardLimitSwitchSource(type, normalOpenOrClose, deviceID, timeoutMs);
  }

  public ErrorCode configForwardLimitSwitchSource(
      RemoteLimitSwitchSource type, LimitSwitchNormal normalOpenOrClose, int deviceID) {
    return thisTalon.configForwardLimitSwitchSource(type, normalOpenOrClose, deviceID);
  }

  public ErrorCode configReverseLimitSwitchSource(
      RemoteLimitSwitchSource type,
      LimitSwitchNormal normalOpenOrClose,
      int deviceID,
      int timeoutMs) {
    return thisTalon.configReverseLimitSwitchSource(type, normalOpenOrClose, deviceID, timeoutMs);
  }

  public ErrorCode configReverseLimitSwitchSource(
      RemoteLimitSwitchSource type, LimitSwitchNormal normalOpenOrClose, int deviceID) {
    return thisTalon.configReverseLimitSwitchSource(type, normalOpenOrClose, deviceID);
  }

  public void overrideLimitSwitchesEnable(boolean enable) {
    thisTalon.overrideLimitSwitchesEnable(enable);
  }

  public ErrorCode configForwardSoftLimitThreshold(int forwardSensorLimit, int timeoutMs) {
    return thisTalon.configForwardSoftLimitThreshold(forwardSensorLimit, timeoutMs);
  }

  public ErrorCode configForwardSoftLimitThreshold(int forwardSensorLimit) {
    return thisTalon.configForwardSoftLimitThreshold(forwardSensorLimit);
  }

  public ErrorCode configReverseSoftLimitThreshold(int reverseSensorLimit, int timeoutMs) {
    return thisTalon.configReverseSoftLimitThreshold(reverseSensorLimit, timeoutMs);
  }

  public ErrorCode configReverseSoftLimitThreshold(int reverseSensorLimit) {
    return thisTalon.configReverseSoftLimitThreshold(reverseSensorLimit);
  }

  public ErrorCode configForwardSoftLimitEnable(boolean enable, int timeoutMs) {
    return thisTalon.configForwardSoftLimitEnable(enable, timeoutMs);
  }

  public ErrorCode configForwardSoftLimitEnable(boolean enable) {
    return thisTalon.configForwardSoftLimitEnable(enable);
  }

  public ErrorCode configReverseSoftLimitEnable(boolean enable, int timeoutMs) {
    return thisTalon.configReverseSoftLimitEnable(enable, timeoutMs);
  }

  public ErrorCode configReverseSoftLimitEnable(boolean enable) {
    return thisTalon.configReverseSoftLimitEnable(enable);
  }

  public void overrideSoftLimitsEnable(boolean enable) {
    thisTalon.overrideSoftLimitsEnable(enable);
  }

  public ErrorCode config_kP(int slotIdx, double value, int timeoutMs) {
    return thisTalon.config_kP(slotIdx, value, timeoutMs);
  }

  public ErrorCode config_kP(int slotIdx, double value) {
    return thisTalon.config_kP(slotIdx, value);
  }

  public ErrorCode config_kI(int slotIdx, double value, int timeoutMs) {
    return thisTalon.config_kI(slotIdx, value, timeoutMs);
  }

  public ErrorCode config_kI(int slotIdx, double value) {
    return thisTalon.config_kI(slotIdx, value);
  }

  public ErrorCode config_kD(int slotIdx, double value, int timeoutMs) {
    return thisTalon.config_kD(slotIdx, value, timeoutMs);
  }

  public ErrorCode config_kD(int slotIdx, double value) {
    return thisTalon.config_kD(slotIdx, value);
  }

  public ErrorCode config_kF(int slotIdx, double value, int timeoutMs) {
    return thisTalon.config_kF(slotIdx, value, timeoutMs);
  }

  public ErrorCode config_kF(int slotIdx, double value) {
    return thisTalon.config_kF(slotIdx, value);
  }

  public ErrorCode config_IntegralZone(int slotIdx, int izone, int timeoutMs) {
    return thisTalon.config_IntegralZone(slotIdx, izone, timeoutMs);
  }

  public ErrorCode config_IntegralZone(int slotIdx, int izone) {
    return thisTalon.config_IntegralZone(slotIdx, izone);
  }

  public ErrorCode configAllowableClosedloopError(
      int slotIdx, int allowableClosedLoopError, int timeoutMs) {
    return thisTalon.configAllowableClosedloopError(slotIdx, allowableClosedLoopError, timeoutMs);
  }

  public ErrorCode configAllowableClosedloopError(int slotIdx, int allowableClosedLoopError) {
    return thisTalon.configAllowableClosedloopError(slotIdx, allowableClosedLoopError);
  }

  public ErrorCode configMaxIntegralAccumulator(int slotIdx, double iaccum, int timeoutMs) {
    return thisTalon.configMaxIntegralAccumulator(slotIdx, iaccum, timeoutMs);
  }

  public ErrorCode configMaxIntegralAccumulator(int slotIdx, double iaccum) {
    return thisTalon.configMaxIntegralAccumulator(slotIdx, iaccum);
  }

  public ErrorCode configClosedLoopPeakOutput(int slotIdx, double percentOut, int timeoutMs) {
    return thisTalon.configClosedLoopPeakOutput(slotIdx, percentOut, timeoutMs);
  }

  public ErrorCode configClosedLoopPeakOutput(int slotIdx, double percentOut) {
    return thisTalon.configClosedLoopPeakOutput(slotIdx, percentOut);
  }

  public ErrorCode configClosedLoopPeriod(int slotIdx, int loopTimeMs, int timeoutMs) {
    return thisTalon.configClosedLoopPeriod(slotIdx, loopTimeMs, timeoutMs);
  }

  public ErrorCode configClosedLoopPeriod(int slotIdx, int loopTimeMs) {
    return thisTalon.configClosedLoopPeriod(slotIdx, loopTimeMs);
  }

  public ErrorCode configAuxPIDPolarity(boolean invert, int timeoutMs) {
    return thisTalon.configAuxPIDPolarity(invert, timeoutMs);
  }

  public ErrorCode configAuxPIDPolarity(boolean invert) {
    return thisTalon.configAuxPIDPolarity(invert);
  }

  public ErrorCode setIntegralAccumulator(double iaccum, int pidIdx, int timeoutMs) {
    return thisTalon.setIntegralAccumulator(iaccum, pidIdx, timeoutMs);
  }

  public ErrorCode setIntegralAccumulator(double iaccum) {
    return thisTalon.setIntegralAccumulator(iaccum);
  }

  public int getClosedLoopError(int pidIdx) {
    return thisTalon.getClosedLoopError(pidIdx);
  }

  public int getClosedLoopError() {
    return thisTalon.getClosedLoopError();
  }

  public double getIntegralAccumulator(int pidIdx) {
    return thisTalon.getIntegralAccumulator(pidIdx);
  }

  public double getIntegralAccumulator() {
    return thisTalon.getIntegralAccumulator();
  }

  public double getErrorDerivative(int pidIdx) {
    return thisTalon.getErrorDerivative(pidIdx);
  }

  public double getErrorDerivative() {
    return thisTalon.getErrorDerivative();
  }

  public void selectProfileSlot(int slotIdx, int pidIdx) {
    thisTalon.selectProfileSlot(slotIdx, pidIdx);
  }

  public double getClosedLoopTarget(int pidIdx) {
    return thisTalon.getClosedLoopTarget(pidIdx);
  }

  public double getClosedLoopTarget() {
    return thisTalon.getClosedLoopTarget();
  }

  public int getActiveTrajectoryPosition() {
    return thisTalon.getActiveTrajectoryPosition();
  }

  public int getActiveTrajectoryPosition(int pidIdx) {
    return thisTalon.getActiveTrajectoryPosition(pidIdx);
  }

  public int getActiveTrajectoryVelocity() {
    return thisTalon.getActiveTrajectoryVelocity();
  }

  public int getActiveTrajectoryVelocity(int pidIdx) {
    return thisTalon.getActiveTrajectoryVelocity(pidIdx);
  }

  public double getActiveTrajectoryHeading() {
    return thisTalon.getActiveTrajectoryHeading();
  }

  public double getActiveTrajectoryArbFeedFwd() {
    return thisTalon.getActiveTrajectoryArbFeedFwd();
  }

  public double getActiveTrajectoryArbFeedFwd(int pidIdx) {
    return thisTalon.getActiveTrajectoryArbFeedFwd(pidIdx);
  }

  public ErrorCode configMotionCruiseVelocity(int sensorUnitsPer100ms, int timeoutMs) {
    return thisTalon.configMotionCruiseVelocity(sensorUnitsPer100ms, timeoutMs);
  }

  public ErrorCode configMotionCruiseVelocity(int sensorUnitsPer100ms) {
    return thisTalon.configMotionCruiseVelocity(sensorUnitsPer100ms);
  }

  public ErrorCode configMotionAcceleration(int sensorUnitsPer100msPerSec, int timeoutMs) {
    return thisTalon.configMotionAcceleration(sensorUnitsPer100msPerSec, timeoutMs);
  }

  public ErrorCode configMotionAcceleration(int sensorUnitsPer100msPerSec) {
    return thisTalon.configMotionAcceleration(sensorUnitsPer100msPerSec);
  }

  public ErrorCode configMotionSCurveStrength(int curveStrength, int timeoutMs) {
    return thisTalon.configMotionSCurveStrength(curveStrength, timeoutMs);
  }

  public ErrorCode configMotionSCurveStrength(int curveStrength) {
    return thisTalon.configMotionSCurveStrength(curveStrength);
  }

  public ErrorCode clearMotionProfileTrajectories() {
    return thisTalon.clearMotionProfileTrajectories();
  }

  public int getMotionProfileTopLevelBufferCount() {
    return thisTalon.getMotionProfileTopLevelBufferCount();
  }

  public ErrorCode pushMotionProfileTrajectory(TrajectoryPoint trajPt) {
    return thisTalon.pushMotionProfileTrajectory(trajPt);
  }

  public ErrorCode startMotionProfile(
      BufferedTrajectoryPointStream stream, int minBufferedPts, ControlMode motionProfControlMode) {
    return thisTalon.startMotionProfile(stream, minBufferedPts, motionProfControlMode);
  }

  public boolean isMotionProfileFinished() {
    return thisTalon.isMotionProfileFinished();
  }

  public boolean isMotionProfileTopLevelBufferFull() {
    return thisTalon.isMotionProfileTopLevelBufferFull();
  }

  public void processMotionProfileBuffer() {
    thisTalon.processMotionProfileBuffer();
  }

  public ErrorCode getMotionProfileStatus(MotionProfileStatus statusToFill) {
    return thisTalon.getMotionProfileStatus(statusToFill);
  }

  public ErrorCode clearMotionProfileHasUnderrun(int timeoutMs) {
    return thisTalon.clearMotionProfileHasUnderrun(timeoutMs);
  }

  public ErrorCode clearMotionProfileHasUnderrun() {
    return thisTalon.clearMotionProfileHasUnderrun();
  }

  public ErrorCode changeMotionControlFramePeriod(int periodMs) {
    return thisTalon.changeMotionControlFramePeriod(periodMs);
  }

  public ErrorCode configMotionProfileTrajectoryPeriod(int baseTrajDurationMs, int timeoutMs) {
    return thisTalon.configMotionProfileTrajectoryPeriod(baseTrajDurationMs, timeoutMs);
  }

  public ErrorCode configMotionProfileTrajectoryPeriod(int baseTrajDurationMs) {
    return thisTalon.configMotionProfileTrajectoryPeriod(baseTrajDurationMs);
  }

  public ErrorCode configMotionProfileTrajectoryInterpolationEnable(boolean enable, int timeoutMs) {
    return thisTalon.configMotionProfileTrajectoryInterpolationEnable(enable, timeoutMs);
  }

  public ErrorCode configMotionProfileTrajectoryInterpolationEnable(boolean enable) {
    return thisTalon.configMotionProfileTrajectoryInterpolationEnable(enable);
  }

  public ErrorCode configFeedbackNotContinuous(boolean feedbackNotContinuous, int timeoutMs) {
    return thisTalon.configFeedbackNotContinuous(feedbackNotContinuous, timeoutMs);
  }

  public ErrorCode configRemoteSensorClosedLoopDisableNeutralOnLOS(
      boolean remoteSensorClosedLoopDisableNeutralOnLOS, int timeoutMs) {
    return thisTalon.configRemoteSensorClosedLoopDisableNeutralOnLOS(
        remoteSensorClosedLoopDisableNeutralOnLOS, timeoutMs);
  }

  public ErrorCode configClearPositionOnLimitF(boolean clearPositionOnLimitF, int timeoutMs) {
    return thisTalon.configClearPositionOnLimitF(clearPositionOnLimitF, timeoutMs);
  }

  public ErrorCode configClearPositionOnLimitR(boolean clearPositionOnLimitR, int timeoutMs) {
    return thisTalon.configClearPositionOnLimitR(clearPositionOnLimitR, timeoutMs);
  }

  public ErrorCode configClearPositionOnQuadIdx(boolean clearPositionOnQuadIdx, int timeoutMs) {
    return thisTalon.configClearPositionOnQuadIdx(clearPositionOnQuadIdx, timeoutMs);
  }

  public ErrorCode configLimitSwitchDisableNeutralOnLOS(
      boolean limitSwitchDisableNeutralOnLOS, int timeoutMs) {
    return thisTalon.configLimitSwitchDisableNeutralOnLOS(
        limitSwitchDisableNeutralOnLOS, timeoutMs);
  }

  public ErrorCode configSoftLimitDisableNeutralOnLOS(
      boolean softLimitDisableNeutralOnLOS, int timeoutMs) {
    return thisTalon.configSoftLimitDisableNeutralOnLOS(softLimitDisableNeutralOnLOS, timeoutMs);
  }

  public ErrorCode configPulseWidthPeriod_EdgesPerRot(
      int pulseWidthPeriod_EdgesPerRot, int timeoutMs) {
    return thisTalon.configPulseWidthPeriod_EdgesPerRot(pulseWidthPeriod_EdgesPerRot, timeoutMs);
  }

  public ErrorCode configPulseWidthPeriod_FilterWindowSz(
      int pulseWidthPeriod_FilterWindowSz, int timeoutMs) {
    return thisTalon.configPulseWidthPeriod_FilterWindowSz(
        pulseWidthPeriod_FilterWindowSz, timeoutMs);
  }

  public ErrorCode getLastError() {
    return thisTalon.getLastError();
  }

  public ErrorCode getFaults(Faults toFill) {
    return thisTalon.getFaults(toFill);
  }

  public ErrorCode getStickyFaults(StickyFaults toFill) {
    return thisTalon.getStickyFaults(toFill);
  }

  public ErrorCode clearStickyFaults(int timeoutMs) {
    return thisTalon.clearStickyFaults(timeoutMs);
  }

  public ErrorCode clearStickyFaults() {
    return thisTalon.clearStickyFaults();
  }

  public int getFirmwareVersion() {
    return thisTalon.getFirmwareVersion();
  }

  public boolean hasResetOccurred() {
    return thisTalon.hasResetOccurred();
  }

  public ErrorCode configSetCustomParam(int newValue, int paramIndex, int timeoutMs) {
    return thisTalon.configSetCustomParam(newValue, paramIndex, timeoutMs);
  }

  public ErrorCode configSetCustomParam(int newValue, int paramIndex) {
    return thisTalon.configSetCustomParam(newValue, paramIndex);
  }

  public int configGetCustomParam(int paramIndex, int timeoutMs) {
    return thisTalon.configGetCustomParam(paramIndex, timeoutMs);
  }

  public int configGetCustomParam(int paramIndex) {
    return thisTalon.configGetCustomParam(paramIndex);
  }

  public ErrorCode configSetParameter(
      ParamEnum param, double value, int subValue, int ordinal, int timeoutMs) {
    return thisTalon.configSetParameter(param, value, subValue, ordinal, timeoutMs);
  }

  public ErrorCode configSetParameter(ParamEnum param, double value, int subValue, int ordinal) {
    return thisTalon.configSetParameter(param, value, subValue, ordinal);
  }

  public ErrorCode configSetParameter(
      int param, double value, int subValue, int ordinal, int timeoutMs) {
    return thisTalon.configSetParameter(param, value, subValue, ordinal, timeoutMs);
  }

  public ErrorCode configSetParameter(int param, double value, int subValue, int ordinal) {
    return thisTalon.configSetParameter(param, value, subValue, ordinal);
  }

  public double configGetParameter(ParamEnum param, int ordinal, int timeoutMs) {
    return thisTalon.configGetParameter(param, ordinal, timeoutMs);
  }

  public double configGetParameter(ParamEnum param, int ordinal) {
    return thisTalon.configGetParameter(param, ordinal);
  }

  public double configGetParameter(int param, int ordinal, int timeoutMs) {
    return thisTalon.configGetParameter(param, ordinal, timeoutMs);
  }

  public double configGetParameter(int param, int ordinal) {
    return thisTalon.configGetParameter(param, ordinal);
  }

  public int getBaseID() {
    return thisTalon.getBaseID();
  }

  public ControlMode getControlMode() {
    return thisTalon.getControlMode();
  }

  public void follow(IMotorController masterToFollow, FollowerType followerType) {
    thisTalon.follow(masterToFollow, followerType);
  }

  public void follow(IMotorController masterToFollow) {
    thisTalon.follow(masterToFollow);
  }

  public void valueUpdated() {
    thisTalon.valueUpdated();
  }

  public ErrorCode configureSlot(SlotConfiguration slot) {
    return thisTalon.configureSlot(slot);
  }

  public ErrorCode configureSlot(SlotConfiguration slot, int slotIdx, int timeoutMs) {
    return thisTalon.configureSlot(slot, slotIdx, timeoutMs);
  }

  public void getSlotConfigs(SlotConfiguration slot, int slotIdx, int timeoutMs) {
    thisTalon.getSlotConfigs(slot, slotIdx, timeoutMs);
  }

  public void getSlotConfigs(SlotConfiguration slot) {
    thisTalon.getSlotConfigs(slot);
  }

  public ErrorCode configureFilter(
      FilterConfiguration filter, int ordinal, int timeoutMs, boolean enableOptimizations) {
    return thisTalon.configureFilter(filter, ordinal, timeoutMs, enableOptimizations);
  }

  public ErrorCode configureFilter(FilterConfiguration filter, int ordinal, int timeoutMs) {
    return thisTalon.configureFilter(filter, ordinal, timeoutMs);
  }

  public ErrorCode configureFilter(FilterConfiguration filter) {
    return thisTalon.configureFilter(filter);
  }

  public void getFilterConfigs(FilterConfiguration filter, int ordinal, int timeoutMs) {
    thisTalon.getFilterConfigs(filter, ordinal, timeoutMs);
  }

  public void getFilterConfigs(FilterConfiguration filter) {
    thisTalon.getFilterConfigs(filter);
  }
}
