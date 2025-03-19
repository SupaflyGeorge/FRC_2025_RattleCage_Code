package frc.robot.subsystems.mechanisms;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    //motors
    private final SparkMax intake1 = new SparkMax(11, MotorType.kBrushless);
    private final SparkMax intake2 = new SparkMax(12, MotorType.kBrushed);
    private final SparkMax intake3 = new SparkMax(13, MotorType.kBrushed);
    //Creates new Intake
    public Intake() {}
    public void set(double speed) {
        intake1.set(speed);
        intake2.set(speed);
        intake3.set(speed);
    }
}


