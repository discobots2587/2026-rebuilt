package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ClimberSubsystemConstants;
import frc.robot.Constants.ShooterSubsystemConstants;

/**
 * Spindexer subsystem that manages the rotation of the carousel and feeder motors.  
 * This class has been extended to expose the encoder position and velocity and  
 * provide a simple jam‑detection mechanism based on sudden changes in encoder  
 * velocity or large current draw.  By monitoring the rate of change of the  
 * encoder velocity, the subsystem can detect when the spindexer stops rotating  
 * smoothly (indicating a jam) and signal for corrective action.  The number of  
 * rotations of the spindexer can be read directly via the provided getter.
 */
public class SpindexerSubsystem extends SubsystemBase {
    private final SparkMax spindexerMotor;
    private final SparkMax feederMotor;
    private final SparkClosedLoopController spindexerController;
    private final RelativeEncoder spindexerEncoder;

    // Thresholds for jam detection. These values should be tuned on the robot.
    private static final double kVelocityDropThresholdRpm = 200.0; // tune to robot
    private static final double kCurrentThresholdAmps = 20.0; // tune to robot
    private static final double kDebounceSeconds = 0.12; // tune to robot

    // Filter to ensure a jam condition persists for a short time before triggering.
    private final Debouncer jamDebouncer = new Debouncer(kDebounceSeconds, DebounceType.kRising);

    private double lastVelocityRpm = 0.0;
    private boolean jamDetected = false;
    private Intake intakeSubsystem;

    /**
     * Constructs a new SpindexerSubsystem.  Motor configuration parameters are loaded
     * from the Configs object, and the encoder position is reset to zero.  The
     * closed loop controller is left in voltage control mode.
     */
    public SpindexerSubsystem(Intake intake) {
        spindexerMotor = new SparkMax(ShooterSubsystemConstants.kSpindexerCanID, SparkMax.MotorType.kBrushless);
        feederMotor = new SparkMax(ShooterSubsystemConstants.kFeederMotorCanId, SparkMax.MotorType.kBrushless);
        spindexerController = spindexerMotor.getClosedLoopController();
        spindexerEncoder = spindexerMotor.getEncoder();
        
        this.intakeSubsystem = intake;
        
        feederMotor.configure(
            Configs.ShooterSubsystem.feederConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
        spindexerMotor.configure(
            Configs.ShooterSubsystem.spindexerConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        spindexerEncoder.setPosition(0);
    }

    /**
     * The current velocity setpoint in RPM.  This is used to determine if a
     * sudden drop in measured velocity constitutes a jam.  It is updated when
     * {@link #setSpindexerVelocity(double)} is called.
     */
    private double spindexerTargetVelocityRpm = 0.0;

    /**
     * Sets a velocity setpoint for the spindexer motor using the closed loop
     * controller in voltage control mode.  The setpoint is stored in RPM for
     * comparison with the actual velocity.
     *
     * @param velocityRpm Desired velocity in rotations per minute.  Positive
     *        values spin forward; negative values spin backward.
     */
    private void setSpindexerVelocity(double velocityRpm) {
        // Use voltage control for the spindexer.  The closed loop controller will
        // convert this into the appropriate motor output voltage.  Passing the
        // setpoint in units of RPM ensures that the velocity reported by the
        // encoder (also in RPM) can be compared directly.
        spindexerController.setSetpoint(velocityRpm, ControlType.kVoltage);
        spindexerTargetVelocityRpm = velocityRpm;
    }

    /**
     * Directly sets the percentage output of the spindexer motor.  This bypasses
     * the closed loop controller and should generally only be used for stopping
     * the motor or for debugging.
     *
     * @param power Fraction of available voltage to apply, from -1.0 to +1.0.
     */
    private void setSpindexerPower(double power) {
        spindexerMotor.set(power);
    }

    /**
     * Directly sets the percentage output of the feeder motor.
     *
     * @param power Fraction of available voltage to apply, from -1.0 to +1.0.
     */
    private void setFeederPower(double power) {
        feederMotor.set(power);
    }

    /**
     * Creates a command that runs the spindexer and optionally the feeder while
     * held.  If {@code spindexer_only} is true, the feeder is not run and the
     * spindexer spins at reduced speed in the reverse direction to clear jams.
     *
     * @param spindexer_only true to run only the spindexer, false to run both
     * @return a command that starts the motors when scheduled and stops them when
     *         the command ends
     */
    public Command runSpindexerCommand(boolean spindexer_only) {
        return this.startEnd(
            () -> {
                // On start: set the appropriate motors running.
                double power = ShooterSubsystemConstants.SpindexerSetpoints.kSpindex;
                if (spindexer_only) {
                    power *= -0.3;
                } else {
                    this.setFeederPower(ShooterSubsystemConstants.FeederSetpoints.kFeed);
                }
                // Convert the desired duty cycle to a voltage setpoint (multiply by 12V).
                this.setSpindexerVelocity(ShooterSubsystemConstants.SpindexerSetpoints.kSpindexVolt);
            }, () -> {
                // On end: stop both motors.
                this.setSpindexerPower(0);
                this.setFeederPower(0);
            }).withName("Spindexing");
    }

    /**
     * Auto spin command that continuously cycles the intake arm up and down
     * while spinning the spindexer for the full duration.
     * Runs until timeout specified in the auto file.
     */
    public Command autoSpinCommand(){
        return new SequentialCommandGroup(
            // Spin spindexer while cycling intake repeatedly
            this.run(() -> {
                setSpindexerPower(ShooterSubsystemConstants.SpindexerSetpoints.kSpindex);
                setFeederPower(ShooterSubsystemConstants.FeederSetpoints.kFeed);
            }).deadlineWith(
                // Run intake arm cycles continuously
                new ParallelRaceGroup(
                    intakeSubsystem.runRaiseCommand().withTimeout(0.5),
                    intakeSubsystem.runLowerCommand().withTimeout(0.5)
                ).repeatedly()
            ),
            // Stop motors when done
            new InstantCommand(() -> {
                setSpindexerPower(0.0);
                setFeederPower(0.0);
            }, this)
        ).withName("AutoSpinWithIntakeCycle");
    }


    

    /**
     * Returns the total number of rotations measured by the spindexer encoder
     * since the position was last reset.  The encoder position is measured in
     * rotations by default, as described in the REV Robotics API documentation.
     *
     * @return the cumulative number of rotations (may be fractional)
     */
    public double getSpindexerRotations() {
        return spindexerEncoder.getPosition();
    }

    /**
     * Returns the current velocity of the spindexer in rotations per minute (RPM).
     * The encoder velocity is reported in RPM by default according to the REV
     * Robotics API.
     *
     * @return current spindexer velocity in RPM
     */
    public double getSpindexerRpm() {
        return spindexerEncoder.getVelocity();
    }

    /**
     * Indicates whether a jam has been detected based on a sustained velocity drop
     * or current spike.  This flag is updated in {@link #periodic()}.
     *
     * @return true if a jam condition has been detected
     */
    public boolean isSpindexerJammed() {
        return jamDetected;
    }

    @Override
    public void periodic() {
        // Publish telemetry to the dashboard.
        SmartDashboard.putNumber("Shooter | Feeder | Applied Output", feederMotor.getAppliedOutput());
        SmartDashboard.putNumber("Shooter | Spindexer | Voltage", spindexerMotor.getBusVoltage());
        SmartDashboard.putNumber("Shooter | Spindexer | Output", spindexerMotor.getAppliedOutput());
        SmartDashboard.putNumber("Shooter | Spindexer | Current", spindexerMotor.getOutputCurrent());
        SmartDashboard.putNumber("Shooter | Spindexer | Velocity", spindexerEncoder.getVelocity());

        // Calculate the change in velocity since the last periodic call.
        double currentVel = spindexerEncoder.getVelocity();
        double deltaVel = currentVel - lastVelocityRpm;
        lastVelocityRpm = currentVel;

        // A jam is indicated by either a sudden drop in velocity from the last
        // measurement or a large increase in current draw.  Both conditions are
        // filtered through a Debouncer so transient spikes are ignored.  The
        // thresholds defined above should be tuned for the mechanism.
        boolean velocityDrop = Math.abs(deltaVel) > kVelocityDropThresholdRpm;
        boolean currentSpike = spindexerMotor.getOutputCurrent() > kCurrentThresholdAmps;
        jamDetected = jamDebouncer.calculate(velocityDrop || currentSpike);

        if (jamDetected) {
            setSpindexerVelocity(-Math.abs(spindexerTargetVelocityRpm));
        }
    }
}



/*

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ShooterSubsystemConstants;

public class SpindexerSubsystem extends SubsystemBase{
    private final SparkMax spindexerMotor;
    private final SparkMax feederMotor;
    private final SparkClosedLoopController spindexerController;
    private final RelativeEncoder spindexerEncoder;
    public SpindexerSubsystem() {
        spindexerMotor = new SparkMax(ShooterSubsystemConstants.kSpindexerCanID, SparkMax.MotorType.kBrushless);
        feederMotor = new SparkMax(ShooterSubsystemConstants.kFeederMotorCanId , SparkMax.MotorType.kBrushless);
        spindexerController = spindexerMotor.getClosedLoopController();
        spindexerEncoder = spindexerMotor.getEncoder();
        feederMotor.configure(
            Configs.ShooterSubsystem.feederConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
        spindexerMotor.configure(
        Configs.ShooterSubsystem.spindexerConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

        spindexerEncoder.setPosition(0);

//   private double flywheelTargetVelocity = 0.0;
//   private boolean runSpindexer =  false;

    }
    private double spindexerTargetVelocity = 0.0;

    private void setSpindexerVelocity(double velocity) {
    // flywheelController.setSetpoint(velocity, ControlType.kMAXMotionVelocityControl);
    spindexerController.setSetpoint(velocity, ControlType.kVoltage); //was duty cycle for percentage
    spindexerTargetVelocity = velocity;
  }

    private void setSpindexerPower(double power){
        spindexerMotor.set(power);
    }
    private void setFeederPower(double power){
        feederMotor.set(power);
    }
    public Command runSpindexerCommand(boolean spindexer_only){
        return this.startEnd(
            () -> {
                double power = ShooterSubsystemConstants.SpindexerSetpoints.kSpindex;
                if(spindexer_only){
                    power *= -0.3;
                }else{
                    this.setFeederPower(ShooterSubsystemConstants.FeederSetpoints.kFeed);
                }
                this.setSpindexerVelocity(ShooterSubsystemConstants.SpindexerSetpoints.kSpindexVolt);
            }, () -> {
                this.setSpindexerPower(0);
                this.setFeederPower(0);
            }).withName("Spindexing");
    }

     @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter | Feeder | Applied Output", feederMotor.getAppliedOutput());

    SmartDashboard.putNumber("Shooter | Spindexer | Voltage", spindexerMotor.getBusVoltage());
    SmartDashboard.putNumber("Shooter | Spindexer | Output", spindexerMotor.getAppliedOutput());
    SmartDashboard.putNumber("Shooter | Spindexer | Current", spindexerMotor.getOutputCurrent());
    SmartDashboard.putNumber("Shooter | Spindexer | Velocity", spindexerEncoder.getVelocity());


   }



    
    
}

*/