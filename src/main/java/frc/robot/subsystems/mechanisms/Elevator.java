// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private final SparkMax leftElevator = new SparkMax(9, MotorType.kBrushless);
  private final SparkMax rightElevator = new SparkMax(10, MotorType.kBrushless);
  
  private final RelativeEncoder armRightMotorEncoder = leftElevator.getEncoder();
  private final RelativeEncoder armLeftMotorEncoder = rightElevator.getEncoder();

  /** Creates a new Arm. */
  public Elevator() {

    final double elevatorMotorRevToDegreesOfElevator = 360.0 / (64 * 5);
    armRightMotorEncoder.setPosition(elevatorMotorRevToDegreesOfElevator);
    armLeftMotorEncoder.setPosition(elevatorMotorRevToDegreesOfElevator);

    final double elevatorWhenLowered = 0;
    armRightMotorEncoder.setPosition(elevatorWhenLowered);
    armLeftMotorEncoder.setPosition(elevatorWhenLowered);
  }

  public void set(double speed) {
    leftElevator.set(speed);
    rightElevator.set(speed);
  }

  public double getPosition() {
    return (armLeftMotorEncoder.getPosition() + armRightMotorEncoder.getPosition()) / 2;
  }

  public double getVelocity() {
    return (armRightMotorEncoder.getVelocity() + armLeftMotorEncoder.getVelocity()) / 10;
  }

  public void configureShuffleboard() {
    Shuffleboard.getTab("Debug (Sensors)").addDouble("Arm Position (Degrees)", () -> getPosition());
  }

  public void resetEncoders() {
    armLeftMotorEncoder.setPosition(0);
    armRightMotorEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
