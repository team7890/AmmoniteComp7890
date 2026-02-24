// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.configs.MotorOutputConfigs;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDs;

public class Shooter extends SubsystemBase {
  private TalonFX objShooter = new TalonFX(MotorIDs.iShooterLeader);
  private TalonFX objFollowShooter = new TalonFX(MotorIDs.iShooterFollower);

  private double dControl, dError;

  private InterpolatingDoubleTreeMap objTreeMap = new InterpolatingDoubleTreeMap();
  

  private StatusCode objTalonFXStatusCode;
  private StatusSignal objStatusSignal;
  public boolean bShooterSpeed = false;
  StatusSignal objStatSig;

  private double distTest;
  
  /** Creates a new Shooter. */
  public Shooter() {

    TalonFXConfiguration objTalonFXConfig = new TalonFXConfiguration();
    objTalonFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    objTalonFXConfig.CurrentLimits.SupplyCurrentLimit = 100.0;
    objTalonFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    objTalonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    objTalonFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.5;
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
    objTreeMap.put(1.0, 2500.0);
    objTreeMap.put(2.0, 3000.0);
    objTreeMap.put(3.0, 4000.0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Speed RPM", getSpeedRPM());
    distTest = SmartDashboard.getNumber("distance test", 1.0);
    SmartDashboard.putNumber("RPM Interp result", getDistance2RPM(distTest));
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
    objShooter.set(dSpeed);

  }

  public void runShooterRPM(double dTargetRPM){
    dControl = dTargetRPM / 5000.0;
    dError = dTargetRPM - getSpeedRPM();
    dControl = dControl + dError * 0.04 / 200.0;
    objShooter.set(dControl);
  }

  public double getDistance2RPM(double dPhotonDist){
    return objTreeMap.get(dPhotonDist);
  } 

  public double getSpeedRPM() {
    objStatSig = objShooter.getVelocity();
    return objStatSig.getValueAsDouble() * 60.0;  
  }
}
