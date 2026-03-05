// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Hopper;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDs;
import frc.robot.Constants.MotorPositions;
import frc.robot.Constants.MotorSpeeds;

public class Pivot extends SubsystemBase {

  private TalonFX objPivot = new TalonFX(MotorIDs.iIntakePivot, "MechCAN");
  private StatusCode objTalonFXStatusCode;

  private Encoder objAbsEncoder;
  private double dOffset = 0.0;

  /** Creates a new Pivot. */
  public Pivot() {
    TalonFXConfiguration objTalonFXConfig = new TalonFXConfiguration();
    objTalonFXConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    objTalonFXConfig.CurrentLimits.SupplyCurrentLimit = 100.0;
    objTalonFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    objTalonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    objTalonFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.15;
    objTalonFXStatusCode = StatusCode.StatusCodeNotInitialized;

    for (int i = 1; i < 5; i++) {
      objTalonFXStatusCode = objPivot.getConfigurator().apply(objTalonFXConfig);
      if (objTalonFXStatusCode.isOK()) break;
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Tilt Angle", absPosition());
    // This method will be called once per scheduler run
  }


  public double absPosition(){
    return (objAbsEncoder.get() * 360 - dOffset);
  }

  public void runTilter(double dSpeed){
    if (absPosition() <= MotorPositions.dPivotMin ) {
      stopTilter();
    }
    else{
      if (absPosition() >= MotorPositions.dPivotMax) {
        stopTilter();
      }
      else objPivot.set(dSpeed);
    }
  }

  public void stopTilter(){
    objPivot.stopMotor();
  }
}