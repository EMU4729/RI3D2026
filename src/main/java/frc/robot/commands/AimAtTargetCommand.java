package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.subsystems.TurretSub;
import frc.robot.utils.TurretTargettingCalc;
import frc.robot.utils.TurretTargettingCalc.TurretCommand;

public class AimAtTargetCommand extends Command{
  TurretSub turret = Subsystems.turret;
  TurretTargettingCalc calc = new TurretTargettingCalc();
  TurretCommand calculatedAim;


  public AimAtTargetCommand(){
    addRequirements(turret);
  }
  
  @Override
  public void initialize() {
    calculatedAim = calc.calcTurret();

    turret.setShooterSpeed(calculatedAim.power());
  }
  
  @Override
  public void execute() {
    calculatedAim = calc.calcTurret();
    
    turret.setShooterSpeed(calculatedAim.power());
    turret.setRotatorAngle(calculatedAim.turretAngle());
    turret.setHoodAngle(calculatedAim.hoodAngle());
  }


  @Override
  public void end(boolean interrupted) {
    turret.setShooterSpeed(0.0);
    turret.setRotatorAngle(Degrees.of(0));
    turret.setHoodAngle(Degrees.of(0));
  }
}
