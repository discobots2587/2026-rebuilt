// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.ShooterSubsystemConstants.FeederSetpoints;
import frc.robot.Constants.ShooterSubsystemConstants.FlywheelSetpoints;
import frc.robot.Constants.IntakeConstants.IntakeSetPoints;
import frc.robot.Constants.ShooterSubsystemConstants;
import edu.wpi.first.wpilibj.Timer;


public class ShooterSubsystem extends SubsystemBase {
  
  // Initialize flywheel SPARKs. We will use MAXMotion velocity control for the flywheel, so we also need to
  // initialize the closed loop controllers and encoders.

  //UPDATE: We are switching to voltage control for the flywheel.
  private SparkMax flywheelMotor = new SparkMax(ShooterSubsystemConstants.kFlywheelMotorCanId, MotorType.kBrushless);
  private SparkClosedLoopController flywheelController = flywheelMotor.getClosedLoopController();
  private RelativeEncoder flywheelEncoder = flywheelMotor.getEncoder();

  private SparkMax flywheelFollowerMotor = new SparkMax(ShooterSubsystemConstants.kFlywheelFollowerMotorCanId, MotorType.kBrushless);
  private SparkClosedLoopController flywheelFollowerController = flywheelMotor.getClosedLoopController();
  private RelativeEncoder flywheelFollowerEncoder = flywheelMotor.getEncoder();

  // Member variables for subsystem state management
  private double flywheelTargetVelocity = 0.0;
  public double flywheelVoltOffset = 0.0;
  
  // PID controller for flywheel voltage control
  private final PIDController flywheelPid;
  private boolean flywheelPidEnabled = false;
  
  // PID gains — primary tuning is kP via SmartDashboard
  private double pidKp  = 0.15;  // Primary tuning parameter
  private double pidKi  = 0.0;   // Set to 0 for simpler control
  private double pidKd  = 0.0;   // Set to 0 for simpler control
  private double pidFF  = 0.0;   // Feedforward (optional)
  
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    /*
     * Apply the appropriate configurations to the SPARKs.
     *
     * kResetSafeParameters is used to get the SPARK to a known state. This
     * is useful in case the SPARK is replaced.
     *run
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
    
    flywheelMotor.configure(
        Configs.ShooterSubsystem.flywheelConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    flywheelFollowerMotor.configure(
        Configs.ShooterSubsystem.flywheelFollowerConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Zero flywheel encoder on initialization
    flywheelEncoder.setPosition(0);
    flywheelFollowerEncoder.setPosition(0);
    
    // Initialize PID controller
    flywheelPid = new PIDController(pidKp, pidKi, pidKd);
    
    // Initialize SmartDashboard with tuning values
    SmartDashboard.putNumber("Shooter/kP",  pidKp);
    SmartDashboard.putNumber("Shooter/kI",  pidKi);
    SmartDashboard.putNumber("Shooter/kD",  pidKd);
    SmartDashboard.putNumber("Shooter/FF",  pidFF);
    SmartDashboard.putBoolean("Shooter/PID Enabled", flywheelPidEnabled);

    System.out.println("---> ShooterSubsystem initialized");
  }

  private boolean isFlywheelAt(double velocity) {
    return MathUtil.isNear(flywheelEncoder.getVelocity(), 
            velocity, FlywheelSetpoints.kVelocityTolerance);
  }

  private boolean isFlywheelFollowerAt(double velocity) {
    return MathUtil.isNear(flywheelFollowerEncoder.getVelocity(), 
            velocity, FlywheelSetpoints.kVelocityTolerance);
  }

  // Main Flywheel Motor Spin Data
  // Trigger: Is the flywheel spinning at the required velocity?
  public final Trigger isFlywheelSpinning = new Trigger(
      () -> isFlywheelAt(FlywheelSetpoints.kShootVoltage) || flywheelEncoder.getVelocity() > FlywheelSetpoints.kShootVoltage
  );
  // public final Trigger isFlywheelSpinningBackwards = new Trigger(
  //     () -> isFlywheelAt(-(FlywheelSetpoints.kShootRpm)) || flywheelEncoder.getVelocity() < -(FlywheelSetpoints.kShootRpm)
  // );
  // Trigger: Is the flywheel stopped?
  public final Trigger isFlywheelStopped = new Trigger(() -> isFlywheelAt(0));

  // Flywheel Follower Motor Spin Data
  // Trigger: Is the flywheel spinning at the required velocity?
  public final Trigger isFlywheelFollowerSpinning = new Trigger(
      () -> isFlywheelFollowerAt(FlywheelSetpoints.kShootVoltage) || flywheelFollowerEncoder.getVelocity() > FlywheelSetpoints.kShootVoltage 
  );
  // public final Trigger isFlywheelFollowerSpinningBackwards = new Trigger(
  //     () -> isFlywheelFollowerAt(-(FlywheelSetpoints.kShootRpm)) || flywheelFollowerEncoder.getVelocity() < -(FlywheelSetpoints.kShootRpm)
  // );
  // Trigger: Is the flywheel stopped?
  public final Trigger isFlywheelFollowerStopped = new Trigger(() -> isFlywheelFollowerAt(0));
  



  /**
   * Drive the flywheels to their set velocity. This will use MAXMotion
   * velocity control which will allow for a smooth acceleration and deceleration to the mechanism's
   * setpoint.
   */
  public void setFlywheelVelocity(double velocity) {
    // flywheelController.setSetpoint(velocity, ControlType.kMAXMotionVelocityControl);
    flywheelController.setSetpoint(velocity, ControlType.kVoltage); //was duty cycle for percentage
    flywheelFollowerController.setSetpoint(velocity, ControlType.kVoltage); //was duty cycle for percentage
    flywheelTargetVelocity = velocity;
  }
  
  /**
   * Command to run the flywheel motors. When the command is interrupted, e.g. the button is released,
   * the motors will stop.
   */

  public Command stopShooter(){
    return this.startEnd(
      () -> {
        this.setFlywheelVelocity(0);
      },
      () -> {
        this.setFlywheelVelocity(0.0);
      }).withName("Stop Flywheel");
   }
  
  public Command runFlywheelCommand() {
    return this.startEnd(
        () -> {
          this.setFlywheelVelocity(FlywheelSetpoints.kShootVoltage);
        },
        () -> {
          this.setFlywheelVelocity(0.0);
        }).withName("Spinning Up Flywheel");
  }

  public Command autoShootCommand(){
        return this.runOnce(() -> {
            final Timer m_time = new Timer();
            m_time.restart();
            ;
            while (!m_time.hasElapsed(6.0)){ //5 might be good 
                setFlywheelVelocity(FlywheelSetpoints.kShootVoltage);
            }; 
            setFlywheelVelocity(0.0);
        });
      }


  /**
   * Meta-command to operate the shooter. The Flywheel starts spinning up and when it reaches
   * the desired speed it starts the Feeder.
   */
  public Command runShooterCommand() {

    //add period to wait for shooter to spin up
    
    //  return this.startEnd(
    //    () -> this.setFlywheelVelocity(FlywheelSetpoints.kShootRpm),
    //    () -> flywheelMotor.stopMotor()
    //  ).until(isFlywheelSpinning).andThen(
    //    this.startEnd(
    //      () -> {
    //        this.setFlywheelVelocity(FlywheelSetpoints.kShootRpm);
    //        this.setFeederPower(FeederSetpoints.kFeed);
    //        // this.setSpindexerPower(SpindexerSetpoints.kSpindex);
    //     }, () -> {
    //        flywheelMotor.stopMotor();
    //        feederMotor.stopMotor();
    //        //spindexerMotor.stopMotor();
    //     })
    //  ).withName("Shooting");

    return this.startEnd(
      () -> {this.setFlywheelVelocity(FlywheelSetpoints.kShootVoltage);
      },() -> {
        this.setFlywheelVelocity(0.0);
      }).withName("Shooting");
  }

// commands for dpad controlled shooter
public Command increaseFlywheelVoltageCommand() {
  return this.runOnce(() -> {
    flywheelVoltOffset += FlywheelSetpoints.kVoltStep;
    flywheelVoltOffset = MathUtil.clamp(flywheelVoltOffset, -12.0, 12.0);
  }).withName("Increasing Flywheel Voltage");
}

public Command decreaseFlywheelVoltageCommand() {
  return this.runOnce(() -> {
    flywheelVoltOffset -= FlywheelSetpoints.kVoltStep;
    flywheelVoltOffset = MathUtil.clamp(flywheelVoltOffset, -12.0, 12.0);
  }).withName("Increasing Flywheel Voltage");
}

public double getFlywheelVoltageOffset() {
    return flywheelVoltOffset;
}

public Command runTeleOpShooterCommand() {
    return this.startEnd(
        () -> this.setFlywheelVelocity(FlywheelSetpoints.kShootVoltage + flywheelVoltOffset),
        () -> this.setFlywheelVelocity(0.0)
    ).withName("Shooting");
}

  // ============ PID CONTROL METHODS ============
  
  /**
   * Enable PID-based flywheel voltage control.
   * When enabled, periodic() will use the PID loop to regulate flywheel voltage.
   * 
   * @param targetVelocity The target velocity/voltage setpoint for the flywheel
   */
  public void enableFlywheelVelocityPID(double targetVelocity) {
    flywheelTargetVelocity = targetVelocity;
    flywheelPidEnabled = true;
    flywheelPid.reset();
    SmartDashboard.putBoolean("Shooter/PID Enabled", flywheelPidEnabled);
  }

  /**
   * Disable PID-based flywheel voltage control.
   * When disabled, setFlywheelVelocity() calls drive the motors directly.
   */
  public void disableFlywheelVelocityPID() {
    flywheelPidEnabled = false;
    flywheelPid.reset();
    SmartDashboard.putBoolean("Shooter/PID Enabled", flywheelPidEnabled);
  }

  /**
   * Update PID gains from values (typically read from SmartDashboard).
   * 
   * @param kp Proportional gain
   * @param ki Integral gain
   * @param kd Derivative gain
   * @param ff Feedforward multiplier (volts per RPM)
   */
  public void setFlywheelPidGains(double kp, double ki, double kd, double ff) {
    pidKp = kp;
    pidKi = ki;
    pidKd = kd;
    pidFF = ff;
    flywheelPid.setPID(pidKp, pidKi, pidKd);
  }

  /**
   * Apply voltage directly to both flywheel motors.
   * Used by PID output and other control modes.
   * 
   * @param voltage Voltage to apply (-12V to 12V)
   */
  private void applyVoltage(double voltage) {
    double clampedVoltage = MathUtil.clamp(voltage, -12.0, 12.0);
    flywheelMotor.setVoltage(clampedVoltage);
    flywheelFollowerMotor.setVoltage(clampedVoltage);
  }

  // ============ DYNAMIC SHOOTER CONTROL (TELEOP) ============
  // This section handles distance-based flywheel and hood control
  
  /*
   * Distance-to-Speed mapping (Voltage vs Distance in meters)
   * TODO: Fill in actual data points from testing
   * Format: double[][] distanceSpeedMap = { {distance, voltage}, {distance, voltage}, ... }
   * Example data structure (COMMENT OUT UNTIL DATA IS AVAILABLE):
   * 
   * private static final double[][] DISTANCE_SPEED_MAP = {
   *   {1.0, -3.0},    // 1m away: 3V
   *   {2.0, -5.0},    // 2m away: 5V
   *   {3.0, -7.0},    // 3m away: 7V
   *   {4.0, -8.5},    // 4m away: 8.5V
   *   {5.0, -10.0},   // 5m away: 10V (max)
   * };
   * 
   * Distance-to-Hood-Angle mapping (degrees vs Distance in meters)
   * private static final double[][] DISTANCE_HOOD_MAP = {
   *   {1.0, 10.0},    // 1m away: 10° hood angle
   *   {2.0, 20.0},    // 2m away: 20° hood angle
   *   {3.0, 30.0},    // 3m away: 30° hood angle
   *   {4.0, 40.0},    // 4m away: 40° hood angle
   *   {5.0, 50.0},    // 5m away: 50° hood angle (max)
   * };
   */

  /**
   * Interpolates a value based on distance using linear interpolation
   * TODO: Uncomment and use once data is available
   * 
   * @param distance Distance from hub in meters
   * @param dataMap 2D array with {distance, value} pairs
   * @return Interpolated value at given distance
   */
  /*
  private double interpolateFromDistance(double distance, double[][] dataMap) {
    // Handle edge cases
    if (distance <= dataMap[0][0]) {
      return dataMap[0][1];
    }
    if (distance >= dataMap[dataMap.length - 1][0]) {
      return dataMap[dataMap.length - 1][1];
    }

    // Find the two points to interpolate between
    for (int i = 0; i < dataMap.length - 1; i++) {
      double d1 = dataMap[i][0];
      double v1 = dataMap[i][1];
      double d2 = dataMap[i + 1][0];
      double v2 = dataMap[i + 1][1];

      if (distance >= d1 && distance <= d2) {
        // Linear interpolation formula: y = y1 + (x - x1) * (y2 - y1) / (x2 - x1)
        double interpolated = v1 + (distance - d1) * (v2 - v1) / (d2 - d1);
        return interpolated;
      }
    }
    return dataMap[dataMap.length - 1][1];
  }
  */

  /**
   * Gets the required flywheel voltage based on distance from hub
   * TODO: Uncomment once data is available
   * 
   * @param distanceFromHub Distance in meters
   * @return Voltage to set flywheel motor (-12V to 12V)
   */
  /*
  public double getFlywheelVoltageFromDistance(double distanceFromHub) {
    return interpolateFromDistance(distanceFromHub, DISTANCE_SPEED_MAP);
  }
  */

  /**
   * Gets the required hood angle based on distance from hub
   * TODO: Uncomment once data is available
   * 
   * @param distanceFromHub Distance in meters
   * @return Hood angle in degrees
   */
  /*
  public double getHoodAngleFromDistance(double distanceFromHub) {
    return interpolateFromDistance(distanceFromHub, DISTANCE_HOOD_MAP);
  }
  */


  @Override
  public void periodic() {
    // ───────────────────────────────────────────────────────────────────────
    // Live-tune PID gains from SmartDashboard
    // ───────────────────────────────────────────────────────────────────────
    double newKp       = SmartDashboard.getNumber("Shooter/kP",          pidKp);
    double newKi       = SmartDashboard.getNumber("Shooter/kI",          pidKi);
    double newKd       = SmartDashboard.getNumber("Shooter/kD",          pidKd);
    double newFF       = SmartDashboard.getNumber("Shooter/FF",          pidFF);
    boolean newEnabled  = SmartDashboard.getBoolean("Shooter/PID Enabled", flywheelPidEnabled);
    
    // Update PID controller if gains have changed
    if (newKp != pidKp || newKi != pidKi || newKd != pidKd) {
      pidKp = newKp;
      pidKi = newKi;
      pidKd = newKd;
      flywheelPid.setPID(pidKp, pidKi, pidKd);
    }
    pidFF = newFF;
    flywheelPidEnabled = newEnabled;

    // ───────────────────────────────────────────────────────────────────────
    // PID Control Logic (when enabled)
    // ───────────────────────────────────────────────────────────────────────
    if (flywheelPidEnabled) {
      double currentVelocity = flywheelEncoder.getVelocity();
      double pidOutput = flywheelPid.calculate(currentVelocity, flywheelTargetVelocity);
      double ffOutput = pidFF * flywheelTargetVelocity;
      double finalVoltage = MathUtil.clamp(pidOutput + ffOutput, -12.0, 12.0);
      applyVoltage(finalVoltage);
      SmartDashboard.putNumber("Shooter | PID Output Voltage", finalVoltage);
    }

    // ───────────────────────────────────────────────────────────────────────
    // Telemetry and Dashboard Logging
    // ───────────────────────────────────────────────────────────────────────
    SmartDashboard.putNumber("Shooter | Flywheel | Applied Output", flywheelMotor.getAppliedOutput());
    SmartDashboard.putNumber("Shooter | Flywheel | Current", flywheelMotor.getOutputCurrent());
    SmartDashboard.putNumber("Shooter | Flywheel Follower | Applied Output", flywheelFollowerMotor.getAppliedOutput());
    SmartDashboard.putNumber("Shooter | Flywheel Follower | Current", flywheelFollowerMotor.getOutputCurrent());

    SmartDashboard.putNumber("Shooter | Flywheel | Target Velocity", flywheelTargetVelocity);
    SmartDashboard.putNumber("Shooter | Flywheel | Actual Velocity", flywheelEncoder.getVelocity());
    SmartDashboard.putNumber("Shooter | Flywheel Follower | Target Velocity", flywheelTargetVelocity);
    SmartDashboard.putNumber("Shooter | Flywheel Follower | Actual Velocity", flywheelFollowerEncoder.getVelocity());

    SmartDashboard.putBoolean("Is Flywheel Spinning", isFlywheelSpinning.getAsBoolean());
    SmartDashboard.putBoolean("Is Flywheel Stopped", isFlywheelStopped.getAsBoolean());
    SmartDashboard.putBoolean("Is Flywheel Follower Spinning", isFlywheelFollowerSpinning.getAsBoolean());
    SmartDashboard.putBoolean("Is Flywheel Follower Stopped", isFlywheelFollowerStopped.getAsBoolean());

    SmartDashboard.putNumber("Shooter | Flywheel | Voltage", flywheelMotor.getBusVoltage());
    SmartDashboard.putNumber("Shooter | Flywheel | Output", flywheelMotor.getAppliedOutput());
    SmartDashboard.putNumber("Shooter | Flywheel | Current", flywheelMotor.getOutputCurrent());
    SmartDashboard.putNumber("Shooter | Flywheel | Velocity", flywheelEncoder.getVelocity());
    SmartDashboard.putNumber("Shooter | Flywheel | Voltage Offset", flywheelVoltOffset);

    SmartDashboard.putNumber("Shooter | Flywheel Follower | Voltage", flywheelFollowerMotor.getBusVoltage());
    SmartDashboard.putNumber("Shooter | Flywheel Follower | Output", flywheelFollowerMotor.getAppliedOutput());
    SmartDashboard.putNumber("Shooter | Flywheel Follower | Current", flywheelFollowerMotor.getOutputCurrent());
    SmartDashboard.putNumber("Shooter | Flywheel Follower | Velocity", flywheelFollowerEncoder.getVelocity());
    SmartDashboard.putNumber("Shooter | Flywheel Follower | Voltage Offset", flywheelVoltOffset);

  }


}

/*TEST THIS CODE 

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// Trigger not used directly; boolean helpers are preferred for compatibility
import frc.robot.Configs;
import frc.robot.Constants.ShooterSubsystemConstants.FlywheelSetpoints;
import frc.robot.Constants.ShooterSubsystemConstants;
import edu.wpi.first.math.controller.PIDController;


public class ShooterSubsystem extends SubsystemBase {

  // Leader flywheel motor, controller, and encoder
  private final SparkMax flywheelMotor =
      new SparkMax(ShooterSubsystemConstants.kFlywheelMotorCanId, MotorType.kBrushless);
  private final SparkClosedLoopController flywheelController =
      flywheelMotor.getClosedLoopController();
  private final RelativeEncoder flywheelEncoder =
      flywheelMotor.getEncoder();

  // FIX #1: Follower controller and encoder now correctly reference the FOLLOWER motor,
  // not the leader. Previously both pointed to flywheelMotor, making follower
  // telemetry a duplicate of leader data and making isFlywheelFollowerAt() meaningless.
  private final SparkMax flywheelFollowerMotor =
      new SparkMax(ShooterSubsystemConstants.kFlywheelFollowerMotorCanId, MotorType.kBrushless);
  private final SparkClosedLoopController flywheelFollowerController =
      flywheelFollowerMotor.getClosedLoopController(); // was flywheelMotor
  private final RelativeEncoder flywheelFollowerEncoder =
      flywheelFollowerMotor.getEncoder();              // was flywheelMotor

  // State variables
  private double flywheelTargetVelocity = 0.0;
  public double flywheelVoltOffset = 0.0;

  // RoboRIO-side PID controller (RPM → voltage output)
  private final PIDController flywheelPid;
  private boolean flywheelPidEnabled = false;

  // PID gains — tunable live via SmartDashboard
  private double pidP  = 0.001;
  private double pidI  = 0.0;
  private double pidD  = 0.0;
  private double pidFF = 0.0; // volts per RPM feedforward multiplier

  public ShooterSubsystem() {
    flywheelMotor.configure(
        Configs.ShooterSubsystem.flywheelConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    flywheelFollowerMotor.configure(
        Configs.ShooterSubsystem.flywheelFollowerConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // NOTE: If flywheelFollowerConfig already sets this motor to hardware-follow
    // the leader via .follow(), you can remove ALL flywheelFollowerController
    // setSetpoint() calls below — hardware following makes them redundant.
    // If it does NOT use hardware follow, the fixed references above ensure
    // the follower gets its own independent setpoints correctly.

    flywheelEncoder.setPosition(0);
    flywheelFollowerEncoder.setPosition(0);

    flywheelPid = new PIDController(pidP, pidI, pidD);

    SmartDashboard.putNumber("Shooter/P",  pidP);
    SmartDashboard.putNumber("Shooter/I",  pidI);
    SmartDashboard.putNumber("Shooter/D",  pidD);
    SmartDashboard.putNumber("Shooter/FF", pidFF);
    SmartDashboard.putBoolean("Shooter/PID Enabled", flywheelPidEnabled);

    System.out.println("---> ShooterSubsystem initialized");
  }

  // ─────────────────────────────────────────────────────────────────────────────
  // Velocity checks
  // ─────────────────────────────────────────────────────────────────────────────

  private boolean isFlywheelAt(double rpm) {
    return MathUtil.isNear(flywheelEncoder.getVelocity(), rpm, FlywheelSetpoints.kVelocityTolerance);
  }

  private boolean isFlywheelFollowerAt(double rpm) {
    return MathUtil.isNear(flywheelFollowerEncoder.getVelocity(), rpm, FlywheelSetpoints.kVelocityTolerance);
  }

  // Simple boolean helpers for subsystem state. We avoid creating Trigger
  // objects here to be compatible with the project's WPILib version and to
  // keep these checks local to the subsystem.
  public boolean isFlywheelSpinning() {
    return flywheelEncoder.getVelocity() >= FlywheelSetpoints.kShootRpm - FlywheelSetpoints.kVelocityTolerance;
}

  public boolean isFlywheelStopped() {
    return isFlywheelAt(0);
  }

  public boolean isFlywheelFollowerSpinning() {
    return isFlywheelFollowerAt(flywheelTargetVelocity) || flywheelFollowerEncoder.getVelocity() > flywheelTargetVelocity;
  }

  public boolean isFlywheelFollowerStopped() {
    return isFlywheelFollowerAt(0);
  }

  // ─────────────────────────────────────────────────────────────────────────────
  // Core motor control
  // ─────────────────────────────────────────────────────────────────────────────

  
  //  * Directly sets both flywheel motors to a voltage setpoint.
  //  * Only called when PID is DISABLED. When PID is enabled, periodic() drives the motors.
  //  *
  //  * FIX #3: The original code had setFlywheelVelocity() and the PID loop in periodic()
  //  * both writing to the motor controller every loop, fighting each other every 20ms.
  //  * Now, direct voltage commands are gated — they only apply when PID is off.
   
  private void applyVoltage(double voltage) {
    double clamped = MathUtil.clamp(voltage, -12.0, 12.0);
    flywheelController.setSetpoint(clamped, ControlType.kVoltage);
    flywheelFollowerController.setSetpoint(clamped, ControlType.kVoltage);
  }

  
  //  Sets the target voltage for direct (non-PID) control.
  //  If PID is enabled, this updates the target RPM used by the PID loop instead.
   
  public void setFlywheelVelocity(double value) {
    flywheelTargetVelocity = value;
    if (!flywheelPidEnabled) {
      applyVoltage(value);
    }
    // If PID is enabled, periodic() will read flywheelTargetVelocity and drive output.
  }

  // ─────────────────────────────────────────────────────────────────────────────
  // Commands
  // ─────────────────────────────────────────────────────────────────────────────

  public Command stopShooterCommand() {
    return this.run(() -> setFlywheelVelocity(0.0))
        .withName("Stop Flywheel");
}

  public Command runFlywheelCommand() {
    return this.startEnd(
        () -> setFlywheelVelocity(FlywheelSetpoints.kShootVoltage),
        () -> setFlywheelVelocity(0.0)
    ).withName("Spinning Up Flywheel");
  }

  // FIX #4: autoShootCommand() previously used a while loop inside runOnce(),
  // which blocks the main robot thread for 6 seconds — freezing all robot code.
  // The correct approach is withTimeout(), which lets the scheduler handle timing.
  public Command autoShootCommand() {
    return runFlywheelCommand()
        .withTimeout(6.0)
        .withName("Auto Shoot");
  }

  public Command runShooterCommand() {
    return this.startEnd(
        () -> setFlywheelVelocity(FlywheelSetpoints.kShootVoltage),
        () -> setFlywheelVelocity(0.0)
    ).withName("Shooting");
  }

  // D-pad voltage trim commands
  public Command increaseFlywheelVoltageCommand() {
    return this.runOnce(() -> {
      flywheelVoltOffset = MathUtil.clamp(
          flywheelVoltOffset + FlywheelSetpoints.kVoltStep, -12.0, 12.0);
    }).withName("Increase Flywheel Voltage");
  }

  public Command decreaseFlywheelVoltageCommand() {
    return this.runOnce(() -> {
      flywheelVoltOffset = MathUtil.clamp(
          flywheelVoltOffset - FlywheelSetpoints.kVoltStep, -12.0, 12.0);
    }).withName("Decrease Flywheel Voltage");
  }

  public Command runTeleOpShooterCommand() {
    return this.startEnd(
        () -> setFlywheelVelocity(FlywheelSetpoints.kShootVoltage + flywheelVoltOffset),
        () -> setFlywheelVelocity(0.0)
    ).withName("TeleOp Shooting");
  }

  public double getFlywheelVoltageOffset() {
    return flywheelVoltOffset;
  }

  // ─────────────────────────────────────────────────────────────────────────────
  // PID velocity control API
  // ─────────────────────────────────────────────────────────────────────────────

  //  Switch to PID velocity control and set a target RPM.
  public void enableFlywheelVelocityPID(double targetRpm) {
    flywheelTargetVelocity = targetRpm;
    flywheelPid.reset();
    flywheelPidEnabled = true;
    SmartDashboard.putBoolean("Shooter/PID Enabled", true);
  }

  //  Switch back to direct voltage control.
  public void disableFlywheelVelocityPID() {
    flywheelPidEnabled = false;
    SmartDashboard.putBoolean("Shooter/PID Enabled", false);
  }

  //  Update PID gains programmatically (also reflects on SmartDashboard).
  public void setFlywheelPidGains(double p, double i, double d, double ff) {
    pidP = p; pidI = i; pidD = d; pidFF = ff;
    flywheelPid.setPID(pidP, pidI, pidD);
    SmartDashboard.putNumber("Shooter/P",  pidP);
    SmartDashboard.putNumber("Shooter/I",  pidI);
    SmartDashboard.putNumber("Shooter/D",  pidD);
    SmartDashboard.putNumber("Shooter/FF", pidFF);
  }

  // ─────────────────────────────────────────────────────────────────────────────
  // Periodic
  // ─────────────────────────────────────────────────────────────────────────────

  @Override
  public void periodic() {
    // Live-tune PID gains from SmartDashboard
    double newP       = SmartDashboard.getNumber("Shooter/P",          pidP);
    double newI       = SmartDashboard.getNumber("Shooter/I",          pidI);
    double newD       = SmartDashboard.getNumber("Shooter/D",          pidD);
    double newFF      = SmartDashboard.getNumber("Shooter/FF",         pidFF);
    boolean newEnabled = SmartDashboard.getBoolean("Shooter/PID Enabled", flywheelPidEnabled);
    
    if (newP != pidP || newI != pidI || newD != pidD) {
      pidP = newP; pidI = newI; pidD = newD;
      flywheelPid.setPID(pidP, pidI, pidD);
    }
    pidFF = newFF;

    
    //Enable or  disable PID based on SmartDashboard toggle
    flywheelPidEnabled = newEnabled;

    // FIX #3 (continued): PID output is the ONLY thing writing to the motor when
    // PID is enabled. setFlywheelVelocity() is blocked from writing in that mode,
    // so there is no longer a conflict between the two control paths.
    if (flywheelPidEnabled) {
      double currentRpm = flywheelEncoder.getVelocity();
      double pidOut     = flywheelPid.calculate(currentRpm, flywheelTargetVelocity);
      double ff         = pidFF * flywheelTargetVelocity;
      double outVolt    = MathUtil.clamp(pidOut + ff, -12.0, 12.0);
      applyVoltage(outVolt);
      SmartDashboard.putNumber("Shooter | Flywheel | PID Voltage", outVolt);
    }

    // Telemetry
    SmartDashboard.putNumber("Shooter | Flywheel | Target Velocity",          flywheelTargetVelocity);
    SmartDashboard.putNumber("Shooter | Flywheel | Actual Velocity",          flywheelEncoder.getVelocity());
    SmartDashboard.putNumber("Shooter | Flywheel Follower | Actual Velocity", flywheelFollowerEncoder.getVelocity());

    SmartDashboard.putNumber("Shooter | Flywheel | Applied Output",           flywheelMotor.getAppliedOutput());
    SmartDashboard.putNumber("Shooter | Flywheel | Current",                  flywheelMotor.getOutputCurrent());
    SmartDashboard.putNumber("Shooter | Flywheel | Voltage",                  flywheelMotor.getBusVoltage());
    SmartDashboard.putNumber("Shooter | Flywheel | Voltage Offset",           flywheelVoltOffset);

    SmartDashboard.putNumber("Shooter | Flywheel Follower | Applied Output",  flywheelFollowerMotor.getAppliedOutput());
    SmartDashboard.putNumber("Shooter | Flywheel Follower | Current",         flywheelFollowerMotor.getOutputCurrent());
    SmartDashboard.putNumber("Shooter | Flywheel Follower | Voltage",         flywheelFollowerMotor.getBusVoltage());

  SmartDashboard.putBoolean("Is Flywheel Spinning",          isFlywheelSpinning());
  SmartDashboard.putBoolean("Is Flywheel Stopped",           isFlywheelStopped());
  SmartDashboard.putBoolean("Is Flywheel Follower Spinning", isFlywheelFollowerSpinning());
  SmartDashboard.putBoolean("Is Flywheel Follower Stopped",  isFlywheelFollowerStopped());
  }
}
 */