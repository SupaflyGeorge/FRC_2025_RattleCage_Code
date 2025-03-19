package frc.robot.subsystems.mechanisms;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
    //motors
    private final SparkMax climb = new SparkMax(16, MotorType.kBrushless);

    //Creates new Intake
    public Climb() {}
    public void set(double speed) {
        climb.set(speed);
        
    }
}