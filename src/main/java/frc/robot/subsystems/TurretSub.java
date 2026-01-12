package frc.robot.subsystems;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Velocity;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.TurretConstants;

public class TurretSub extends SubsystemBase {
  private final TalonFX rotatorMotor;
  private final TalonFXConfiguration rotatorMotorConfig;
  private final TalonFX shooterMotor;
  private final TalonFXConfiguration shooterMotorConfig;
  private final TalonFX hoodMotor;
  private final TalonFXConfiguration hoodMotorConfig;
  
  /** Creates new TurretSub */
  public TurretSub() {
    // Initialize each of the motors + encoders needed
    rotatorMotor = new TalonFX(TurretConstants.ROTATOR_CANID);
    rotatorMotorConfig = new TalonFXConfiguration();
    rotatorMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rotatorMotorConfig.Feedback.SensorToMechanismRatio = 1; // DriveConstants.DRIVE_GEAR_RATIO;
    rotatorMotorConfig.Slot0.kP = TurretConstants.ROTATOR_P;
    rotatorMotorConfig.Slot0.kI = TurretConstants.ROTATOR_I;
    rotatorMotorConfig.Slot0.kD = TurretConstants.ROTATOR_D;
    
    shooterMotor = new TalonFX(TurretConstants.SHOOTER_CANID);
    shooterMotorConfig = new TalonFXConfiguration();
    shooterMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake; // Coast?

    // Controls the pitch of the shooter, by changing the angle of hood
    hoodMotor = new TalonFX(TurretConstants.SHOOTER_CANID);
    hoodMotorConfig = new TalonFXConfiguration();
    hoodMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    hoodMotorConfig.Feedback.SensorToMechanismRatio = 1;
    hoodMotorConfig.Slot0.kP = TurretConstants.HOOD_P;
    hoodMotorConfig.Slot0.kI = TurretConstants.HOOD_I;
    hoodMotorConfig.Slot0.kD = TurretConstants.HOOD_D;
    
    setupSmartDash();
  }

  // TODO: not sure whether this will automatically use the PID control, or need to set up close loop control
  public void setRotatorAngle(Angle newRotatorAngle) {
    rotatorMotor.setPosition(newRotatorAngle);
  }

  public Angle getRotatorAngle() {
    return rotatorMotor.getPosition().getValue();
    // return Angle.ofBaseUnits(0, Radians);
  }

  // Set the amount of power into the shooter wheels (-1.0 to 1.0)
  public void setShooterSpeed(Double shooterSpeed) {
    shooterMotor.set(shooterSpeed);
  }

  public Double getShooterSpeed() {
    return shooterMotor.get();
  }

  public void setHoodAngle(Angle newHoodAngle) {
    hoodMotor.setPosition(newHoodAngle);
  }

  public Angle getHoodAngle() {
    return hoodMotor.getPosition().getValue();
  }

  public void resetEncoders() {
    rotatorMotor.setPosition(0);
    shooterMotor.setPosition(0);
  }

  @Override
  public void simulationPeriodic() {
    // TODO: handle simulation (doesn't have whole module support like the swerve module)
  }

  public void setupSmartDash() {
    SmartDashboard.putData("Turret Sub", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Turret");

        builder.addDoubleProperty("Rotator Angle (rad)", () -> rotatorMotor.getPosition().getValueAsDouble(), null);
        builder.addDoubleProperty("Rotator Angular Velocity (rad/s)", () -> rotatorMotor.getVelocity().getValueAsDouble(), null);
        
        builder.addDoubleProperty("Hood Angle (rad)", () -> hoodMotor.getPosition().getValueAsDouble(), null);
     
        builder.addDoubleProperty("Shooter Power", () -> shooterMotor.get(), null);
      }
    });
  }
}
