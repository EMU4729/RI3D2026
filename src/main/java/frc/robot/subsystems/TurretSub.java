package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Velocity;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Subsystems;
import frc.robot.constants.TurretConstants;
import frc.robot.utils.TurretTargettingCalc;

public class TurretSub extends SubsystemBase {
  private final TalonFX rotatorMotor;
  private final TalonFXConfiguration rotatorMotorConfig;
  private final TalonFX shooterMotor;
  private final TalonFXConfiguration shooterMotorConfig;
  private final TalonFX hoodMotor;
  private final TalonFXConfiguration hoodMotorConfig;

  private final PositionVoltage rotatorController;
  private final VelocityVoltage shooterController;
  private final PositionVoltage hoodController;

  private SlewRateLimiter rotatorLimit = new SlewRateLimiter(5);
  private SlewRateLimiter shooterLimiter = new SlewRateLimiter(25);
  private SlewRateLimiter hooLimiter = new SlewRateLimiter(1);

  private Angle targetRotatorAngle = Rotations.of(0);
  private double simRotatorSpeed = 0;
  private AngularVelocity targetShooterSpeed = RotationsPerSecond.of(0);
  private Angle targetHoodAngle = Rotations.of(0);

  private final TalonFXSimState rotatorMotorSim;
  private final TalonFXSimState hoodMotorSim;
  
  /** Creates new TurretSub */
  public TurretSub() {
    // Initialize each of the motors + encoders needed
    rotatorMotor = new TalonFX(TurretConstants.ROTATOR_CANID);
    rotatorMotorConfig = new TalonFXConfiguration();
    rotatorMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    rotatorMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.5;
    rotatorMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    rotatorMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.5;
    rotatorMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rotatorMotorConfig.Feedback.SensorToMechanismRatio = TurretConstants.rotatorMotorRatio; // DriveConstants.DRIVE_GEAR_RATIO;
    rotatorMotorConfig.Slot0.kP = TurretConstants.ROTATOR_P;
    rotatorMotorConfig.Slot0.kI = TurretConstants.ROTATOR_I;
    rotatorMotorConfig.Slot0.kD = TurretConstants.ROTATOR_D;
    rotatorMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    // rotatorMotorConfig.SoftwareLimitSwitch.
    rotatorMotor.getConfigurator().apply(rotatorMotorConfig);
    rotatorController = new PositionVoltage(0).withSlot(0);
    rotatorController.Velocity = 1;

    shooterMotor = new TalonFX(TurretConstants.SHOOTER_CANID);
    shooterMotorConfig = new TalonFXConfiguration();
    shooterMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast; // Coast?
    shooterMotorConfig.Feedback.SensorToMechanismRatio = TurretConstants.shooterMotorRatio;
    shooterMotorConfig.Slot0.kP = TurretConstants.SHOOTER_P;
    shooterMotorConfig.Slot0.kI = TurretConstants.SHOOTER_I;
    shooterMotorConfig.Slot0.kD = TurretConstants.SHOOTER_D;
    shooterMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    shooterMotor.getConfigurator().apply(shooterMotorConfig);
    shooterController = new VelocityVoltage(0).withSlot(0);

    // Controls the pitch of the shooter, by changing the angle of hood
    hoodMotor = new TalonFX(TurretConstants.HOOD_CANID);
    hoodMotorConfig = new TalonFXConfiguration();
    hoodMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    hoodMotorConfig.Feedback.SensorToMechanismRatio = TurretConstants.hoodMotorRatio;
    hoodMotorConfig.Slot0.kP = TurretConstants.HOOD_P;
    hoodMotorConfig.Slot0.kI = TurretConstants.HOOD_I;
    hoodMotorConfig.Slot0.kD = TurretConstants.HOOD_D;
    hoodMotor.getConfigurator().apply(hoodMotorConfig);
    hoodController = new PositionVoltage(0).withSlot(0);
    hoodController.Velocity = 1;


    rotatorMotorSim = rotatorMotor.getSimState();
    hoodMotorSim = hoodMotor.getSimState();

    setupSmartDash();
  }

  // TODO: not sure whether this will automatically use the PID control, or need to set up close loop control
  // Setting this up like a linear system may work better in terms of preventing overrotation :)
  public void setRotatorAngle(Angle newRotatorAngle) {
    
    targetRotatorAngle = newRotatorAngle;
    
  }
  
  public Angle getRotatorAngle() {
    return rotatorMotor.getPosition().getValue();
  }
  
  // Set the amount of power into the shooter wheels (-1.0 to 1.0)
  public void setShooterSpeed(AngularVelocity shooterSpeed) {
    targetShooterSpeed = shooterSpeed;
  }

  public double getShooterSpeed() {
    if(Robot.isSimulation()){return targetShooterSpeed.in(RotationsPerSecond);}
    return shooterMotor.getVelocity().getValue().in(RotationsPerSecond);
  }

  public void setHoodAngle(Angle newHoodAngle) {
    targetHoodAngle = newHoodAngle;
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
    rotatorMotorSim.setRawRotorPosition(targetRotatorAngle.in(Rotations) / TurretConstants.rotatorMotorRatio);
    hoodMotorSim.setRawRotorPosition(targetHoodAngle.in(Rotations) / TurretConstants.hoodMotorRatio);
  }

  public void setupSmartDash() {
    SmartDashboard.putData("Turret Sub", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Turret");

        builder.addDoubleProperty("Rotator Angle (rad)", () -> Robot.isSimulation() ? targetRotatorAngle.in(Degrees) : rotatorMotor.getPosition().getValueAsDouble(), null);
        builder.addDoubleProperty("Rotator Angular Velocity (rad/s)", () -> Robot.isSimulation() ? simRotatorSpeed : rotatorMotor.getVelocity().getValueAsDouble(), null);
        
        builder.addDoubleProperty("Hood Angle (rad)", () -> Robot.isSimulation() ? targetHoodAngle.in(Degrees) : hoodMotor.getPosition().getValueAsDouble(), null);
     
        builder.addDoubleProperty("Shooter Power", () -> getShooterSpeed(), null);
        builder.addDoubleProperty("sim shooter Power", () -> targetShooterSpeed.in(RotationsPerSecond), null);
      }
    });
  }


  
  @Override
  public void periodic() {
    
    targetRotatorAngle = Rotations.of(rotatorLimit.calculate(targetRotatorAngle.in(Rotations)));
    // simRotatorSpeed = 0; // TODO: start simulating this once PID is simulated
    rotatorMotor.setControl(
        hoodController
            .withPosition(targetRotatorAngle));
    System.out.println("TST:"+targetRotatorAngle.in(Rotations));
    
    double shooterSpeed = shooterLimiter.calculate(targetShooterSpeed.in(RotationsPerSecond));
    shooterMotor.setControl(
        shooterController
            .withVelocity(RotationsPerSecond.of(shooterSpeed)));

    double newHoodAngle = hooLimiter.calculate(targetHoodAngle.in(Rotations));
    hoodMotor.setControl(
        hoodController
            .withPosition(newHoodAngle));


    Subsystems.nav.drawFieldObject("Turret", 
        new Pose2d(
            TurretTargettingCalc.turretTranslation.getTranslation().toTranslation2d(), 
            Rotation2d.fromRadians(getRotatorAngle().in(Radians))),
        true);
  }
}
