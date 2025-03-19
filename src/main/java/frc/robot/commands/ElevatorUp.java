package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanisms.Elevator;

public class ElevatorUp extends Command {
    public final Elevator elevatorSubsystem;
    public final double speed;

    //Creates new Feed
    public ElevatorUp(Elevator elevatorSubsystem, double speed) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.speed = speed;

        addRequirements(elevatorSubsystem);
    }

    //Called everytime the scheduler runs while the command is scheduled
    @Override
    public void execute() {
        elevatorSubsystem.set(speed);
    }

    //Called once the command ends or is interrupted
    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.set(0);
    }

    //Returns true when the command should end
    @Override
    public boolean isFinished() {
        return false;
    }
}
