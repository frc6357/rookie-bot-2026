package frc.robot.subsystems.indexer;

/* NOTE: This code and these comments were made by an inexperienced rookie.
 * Tread carefuly.
 */

import edu.wpi.first.wpilibj2.command.SubsystemBase;
// imports the variable for the motor port
import static frc.robot.Ports.IndexerPorts.kIndexerMotor;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
// imports the class that helps control the motor
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Indexer extends SubsystemBase {
    // motor initialization
    SparkFlex indexerMotor = new SparkFlex(kIndexerMotor.ID, MotorType.kBrushless);
    SparkClosedLoopController indexerMotorController = indexerMotor.getClosedLoopController();
    Indexer() {
        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kBrake);
        indexerMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void runMotor() {

    }

    @Override
    public void periodic() {

    }
    
}
