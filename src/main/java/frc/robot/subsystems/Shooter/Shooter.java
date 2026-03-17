// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.configs.MotorOutputConfigs;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private TalonFX objShooter; // = new TalonFX(MotorIDs.iShooterLeader, "MechCAN");
  private TalonFX objFollowShooter; //  = new TalonFX(MotorIDs.iShooterFollower, "MechCAN");

  private double dControl, dError;

  private InterpolatingDoubleTreeMap objTreeMap = new InterpolatingDoubleTreeMap();
  
  private StatusCode objTalonFXStatusCode;
  private StatusSignal objStatusSignal;
  public boolean bShooterSpeed = false;
  StatusSignal objStatSig;

  private double distTest;
  private double dTempRPM;

  // Control shooter as a flywheel with voltage compensation to ensure consistent speed
  final VelocityVoltage m_velocity  = new VelocityVoltage(0);

  
  /** Creates a new Shooter. */
  public Shooter() {
    objShooter = new TalonFX(Constants.MotorIDs.iShooterLeader, Constants.mechCanBus);
    objFollowShooter =  new TalonFX(Constants.MotorIDs.iShooterFollower, Constants.mechCanBus);
    m_velocity.Slot = 0;

    TalonFXConfiguration objTalonFXConfig = new TalonFXConfiguration();
    objTalonFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    objTalonFXConfig.CurrentLimits.SupplyCurrentLimit = 60.0;
    objTalonFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    objTalonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    objTalonFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.1;
    objTalonFXStatusCode = StatusCode.StatusCodeNotInitialized;

    for (int i = 1; i < 5; i++) {
      objTalonFXStatusCode = objShooter.getConfigurator().apply(objTalonFXConfig);
      if (objTalonFXStatusCode.isOK()) break;
    }

     for (int i = 1; i < 5; i++) {
      objTalonFXStatusCode = objFollowShooter.getConfigurator().apply(objTalonFXConfig);
      if (objTalonFXStatusCode.isOK()) break;
     }

  
    objFollowShooter.setControl(new Follower(objShooter.getDeviceID(), MotorAlignmentValue.Opposed));

    // === TREE MAP CONFIG === \\
     // TODO: FIX VALUES
     // These keys are in meters not feet

    //objTreeMap.put(1.0, 3250.0);
    objTreeMap.put(30.0, 0.0);

    // These are for more points later
    // objTreeMap.put(42.0,3350.0);
    // objTreeMap.put(45.0,3600.0);
    // objTreeMap.put(47.0,3660.0);
    
    objTreeMap.put(40.0, 3200.0);
    objTreeMap.put(42.0,3350.0);

    //Same here
//     objTreeMap.put(55.0,3600.0);
//     objTreeMap.put(,3660.0);

    objTreeMap.put(50.0, 3600.0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Speed RPM", getSpeedRPM());
    distTest = SmartDashboard.getNumber("distance test", 1.0);
    SmartDashboard.putNumber("RPM Interp result", getDistance2RPM(distTest));
    dTempRPM = SmartDashboard.getNumber("Test RPM", 3500.0);
    SmartDashboard.putNumber("Test RPM", dTempRPM);
  }

  public BooleanSupplier bsShooterFast(){
    if (getSpeedRPM() > 1000.0) {
      return () -> true;
    }
    
    else return () -> false;
  }
   
  public void stopShooter(){
    objShooter.stopMotor();
  }

  public void runShooter(double dSpeed){
    //objShooter.set(dSpeed);
    //Change to velocity control with v comp
    objShooter.setControl(m_velocity.withVelocity(dSpeed).withSlot(0));
  }

  public void runShooterRPM(double dTargetRPM){
    // dTargetRPM = dTempRPM;
    dControl = dTargetRPM / 5000.0;
    dError = dTargetRPM - getSpeedRPM();
    dControl = dControl + dError * 0.2 / 200.0; // was 0.04
    objShooter.set(dControl);
  }

  public double getDistance2RPM(double dPhotonDist){
    return objTreeMap.get(dPhotonDist);
  } 

  public double getSpeedRPM() {
    objStatSig = objShooter.getVelocity();
    return objStatSig.getValueAsDouble() * 60.0;  
  }

  public Command shooterRunning(double dSpeed){
    return run(() -> objShooter.set(dSpeed));
  }
}
