package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.subsystems.TurretSub;

public class ShootAtTargetCommand extends Command {
  private LinearVelocity speed;

  public ShootAtTargetCommand(LinearVelocity speed){
    this.speed = speed;
    addRequirements(Subsystems.feeder);
  }
  
  @Override
  public void initialize() {
    if (Math.abs(Subsystems.turret.getShooterSpeed()) > 0.01){
      Subsystems.feeder.setSpeed(speed);
    }
  }

  @Override
  public void end(boolean interrupted) {
    Subsystems.feeder.setSpeed(MetersPerSecond.of(0));
  }
}
