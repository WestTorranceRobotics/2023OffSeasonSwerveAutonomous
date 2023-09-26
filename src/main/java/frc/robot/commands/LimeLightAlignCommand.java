// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveDriveTrain.DriveTrain;

public class LimeLightAlignCommand extends CommandBase {
  /** Creates a new LimeLightAllignCommand. */
  DriveTrain driveTrain;
  LimelightSubsystem limelightSubsystem;
  
  public LimeLightAlignCommand(DriveTrain drive,LimelightSubsystem lime) {
    driveTrain = drive;
    limelightSubsystem = lime;
    addRequirements(drive,lime);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xtranslate = limelightSubsystem.alignmentCaculation();
    //double rotation = driveTrain.caculateZeroAngle();
    driveTrain.setSwerveDrive(xtranslate, 0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.setSwerveDrive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
