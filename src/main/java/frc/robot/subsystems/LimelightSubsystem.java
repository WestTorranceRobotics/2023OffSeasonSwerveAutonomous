// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.net.CacheRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new Limelight. */
  ShuffleboardTab limelightTab = Shuffleboard.getTab("Limelight");
  private GenericEntry SBIsFinished =  limelightTab.add("Is Finished?",false).withPosition(0, 0).getEntry();
  private GenericEntry SBReturnSpeed =  limelightTab.add("Speed?",0).withPosition(1, 0).getEntry();
  private GenericEntry SBRotationKp = limelightTab.add("Rotation kP",0.002).withPosition(0, 1).getEntry();
  private GenericEntry SBRotationKi = limelightTab.add("Rotation kI",0).withPosition(1, 1).getEntry();
  private GenericEntry SBRotationKd = limelightTab.add("Rotation kD",0.0005).withPosition(2, 1).getEntry();
 
  private PIDController limePIDController;
  private boolean isFinished;
  double a_kp;
  double a_ki;
  double a_kd;
  public LimelightSubsystem() {
    a_kp = 0.1;
    a_ki = 0;
    a_kd = 0.05;
    limePIDController = new PIDController(0.1, 0, 0.05);
    limePIDController.setTolerance(2.5);
    


  }

  
  public double getTX() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  }

  public double getTY() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  }

  public double getTV() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
  }

  public void setPipeline(int pipline){

    NetworkTableInstance.getDefault().getTable("limelight").getEntry("getpipe").setInteger(pipline);
  }

  public double alignmentCaculation(){
    double calculation = MathUtil.clamp(limePIDController.calculate(getTX(),-16.7),-0.75,0.75);
    return calculation;
  }


  @Override
  public void periodic() {
    if ((a_kp != SBRotationKp.getDouble(0)) || (a_ki != SBRotationKi.getDouble(0)) || (a_kd != SBRotationKd.getDouble(0))){
      a_kp = SBRotationKp.getDouble(0);
      a_ki = SBRotationKi.getDouble(0);
      a_kd = SBRotationKd.getDouble(0);
      limePIDController.setPID(a_kp, a_ki, a_kd);
    }
    // This method will be called once per scheduler run
  }
}
