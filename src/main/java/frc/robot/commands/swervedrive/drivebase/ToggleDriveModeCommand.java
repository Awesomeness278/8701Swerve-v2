// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class ToggleDriveModeCommand extends CommandBase {
  /** Creates a new ToggleDriveModeCommand. */
  Command c1;
  Command c2;
  SwerveSubsystem subsystem;
  public ToggleDriveModeCommand(Command c1, Command c2, SwerveSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.c1 = c1;
    this.c2 = c2;
    this.subsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Constants.toggle = !Constants.toggle;
    if(Constants.toggle){
      subsystem.setDefaultCommand(c2);
    }else{
      subsystem.setDefaultCommand(c1);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
