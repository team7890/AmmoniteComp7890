// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorIDs;

public class Feeder extends SubsystemBase {

  private TalonFX objFeeder = new TalonFX(MotorIDs.iFeederLeader);
  private TalonFX objFollowFeeder = new TalonFX(MotorIDs. iFeederFollower);
  private StatusCode objFeederStatusCode;
  private StatusSignal objStatusSignal;
  private int iCount = 0;
  /** Creates a new Feeder. */
  
  public Feeder() {

    TalonFXConfiguration objTalonFXConfig = new TalonFXConfiguration();
    objTalonFXConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    //objTalonFXConfig.CurrentLimits.SupplyCurrentLimit = 500.0;
    objTalonFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    objTalonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    objTalonFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.5;
    objFeederStatusCode = StatusCode.StatusCodeNotInitialized;


    for (int i = 1; i < 5; i++) {
      objFeederStatusCode = objFeeder.getConfigurator().apply(objTalonFXConfig);
      if (objFeederStatusCode.isOK()) break;
    }

    for (int i = 1; i < 5; i++) {
      objFeederStatusCode = objFeeder.getConfigurator().apply(objTalonFXConfig);
      if (objFeederStatusCode.isOK()) break;
    }

    objFollowFeeder.setControl(new Follower(objFeeder.getDeviceID(), MotorAlignmentValue.Opposed));
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void stopFeeder(){
    objFeeder.stopMotor();

  }

  public void runFeeder(double dSpeed){
    objFeeder.set(dSpeed);
  
  }
}
