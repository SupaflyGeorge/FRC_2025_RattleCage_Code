// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanisms.Pivot;;

public class PivotSet extends Command {
  private final Pivot pivotSubsystem;
  private final DoubleSupplier speedSupplier;
  
  /** Creates a new ArmSet. */
  public PivotSet(Pivot pivotSubsystem, double speedSupplier) {
    this.pivotSubsystem = pivotSubsystem;
    this.speedSupplier = () -> -speedSupplier;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pivotSubsystem);
  }

 


// Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivotSubsystem.set(speedSupplier.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pivotSubsystem.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}