// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Hopper;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.MotorIDs;
import frc.robot.Constants.MotorPositions;
import frc.robot.Constants.MotorSpeeds;

public class Pivot extends SubsystemBase {

  private TalonFX objPivot = new TalonFX(MotorIDs.iIntakePivot, "MechCAN");
  private CANcoder objIntakeEncoder = new CANcoder(MotorIDs.iEncoderPivotID, "MechCAN");
  private StatusCode objTalonFXStatusCode;
  private double dReportedPos;

  // private CANcoder objAbsEncoder;
  private double dOffset = 0.0;

  /** Creates a new Pivot. */
  public Pivot() {
    TalonFXConfiguration objTalonFXConfig = new TalonFXConfiguration();
    objTalonFXConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    objTalonFXConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
    objTalonFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    objTalonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    objTalonFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.15;
    objTalonFXStatusCode = StatusCode.StatusCodeNotInitialized;

    for (int i = 1; i < 5; i++) {
      objTalonFXStatusCode = objPivot.getConfigurator().apply(objTalonFXConfig);
      if (objTalonFXStatusCode.isOK()) break;
    }
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumberArray("Tilt Angle", absPosition());
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake Position", getIntakePosition());
  }


  // public StatusSignal<Angle> absPosition(){
  //   return (objAbsEncoder.getAbsolutePosition());
  // }

  // public void runTilter(double dSpeed){
  //   if (absPosition() <= MotorPositions.dPivotMin ) {
  //     stopTilter();
  //   }
  //   else{
  //     if (absPosition() >= MotorPositions.dPivotMax) {
  //       stopTilter();
  //     }
  //     else objPivot.set(dSpeed);
  //   }
  // }

  public void stopPivot(){
    objPivot.stopMotor();
  }

  public void runPivot(double dSpeed){
    objPivot.set(dSpeed);
  }

  public double getIntakePosition() {
    dReportedPos = objIntakeEncoder.getAbsolutePosition().getValueAsDouble();

    if (dReportedPos > 0.5 && dReportedPos <= 1.0) {
      dReportedPos = dReportedPos - 0.4;
    }

    else {
      dReportedPos = dReportedPos + 0.6;
    }
    return dReportedPos;
  }

  public void agitatePivot(){
      if(getIntakePosition() >= 0.60){
        objPivot.set(0.1);
      }
      else{
        if (getIntakePosition() <= 0.67) {
          objPivot.set(-0.1);
        }
      }
  }
}