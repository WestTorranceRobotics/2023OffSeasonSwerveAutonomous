// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAnalogSensor.Mode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CubeShooter extends SubsystemBase {
  /** Creates a new CubeShooter. */
  ShuffleboardTab shootertab = Shuffleboard.getTab("Cube Shooter");
  private GenericEntry SBShooterTopSpeed= shootertab.add("Shooter Top Speed", 0).withPosition(0, 0).getEntry();
  private GenericEntry SBShooterBottomSpeed= shootertab.add("Shooter Bottom Speed", 0).withPosition(1, 0).getEntry();
  private GenericEntry SBShooterBottomTargetSpeed= shootertab.add("Shooter Bottom Target Speed", 0).withPosition(0, 1).getEntry();
  private GenericEntry SBShooterTopTargetSpeed= shootertab.add("Shooter Top Target Speed", 0).withPosition(1, 1).getEntry();
  private GenericEntry SBShooterBottomTargetSpeedPID= shootertab.add("Shooter Bottom PID Target Speed", 0).withPosition(0, 2).getEntry();
  private GenericEntry SBShooterTopTargetSpeedPID = shootertab.add("Shooter Top PID Target Speed", 0).withPosition(1, 2).getEntry();


  WPI_TalonSRX shooter_top;
  CANSparkMax shooter_bottom;
  PIDController shooterTopPIDController;
  PIDController shooterBottomPIDController;


  public CubeShooter() {
    shooter_top = new WPI_TalonSRX(30);
    shooter_bottom = new CANSparkMax(20,MotorType.kBrushless);
    shooter_bottom.restoreFactoryDefaults();

    shooterTopPIDController = new PIDController(0, 0, 0);
    shooterBottomPIDController = new PIDController(0, 0, 0);

  }

  public void setShooterSpeedTop(double speed){
    shooter_top.set(speed);
  }

  public void setShooterSpeedBottom(double speed){
    shooter_bottom.set(speed);
  }

  public void setShooterSpeedTopPID(double speed){
    double TargetSpeed = shooterTopPIDController.calculate(shooter_top.getSelectedSensorVelocity(),speed);
    shooter_top.set(TargetSpeed);
  }

  public void setShooterSpeedBottomPID(double speed){
    double TargetSpeed = shooterBottomPIDController.calculate(shooter_bottom.getEncoder().getVelocity(),speed);
    shooter_bottom.set(TargetSpeed);
  }

  public double getTargetSpeedBottom(){
    return SBShooterBottomTargetSpeed.getDouble(0);
  }  
  public double getTargetSpeedTop(){
    return SBShooterTopTargetSpeed.getDouble(0);
  }

  public double getTargetPIDSpeedBottom(){
    return SBShooterBottomTargetSpeedPID.getDouble(0);
  }  
  public double getTargetPIDSpeedTop(){
    return SBShooterTopTargetSpeedPID.getDouble(0);
  }

  
  
  @Override
  public void periodic() {
    SBShooterBottomSpeed.setDouble(shooter_bottom.getEncoder().getVelocity());
    SBShooterTopSpeed.setDouble(shooter_top.getSelectedSensorVelocity());
    // This method will be called once per scheduler run
  }
}
