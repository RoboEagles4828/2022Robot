package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.*;
import frc.robot.Constants.*;

public class DriveTrain {
    private MotorControllerGroup left;
    private MotorControllerGroup right;
    private DifferentialDrive dt;

    public WPI_TalonFX front_left = new WPI_TalonFX(Ports.front_left_port);
    public WPI_TalonFX back_left = new WPI_TalonFX(Ports.back_left_port);
    public WPI_TalonFX front_right = new WPI_TalonFX(Ports.front_right_port);
    public WPI_TalonFX back_right = new WPI_TalonFX(Ports.back_right_port);

    public DriveTrain(){
        left = new MotorControllerGroup(front_left, back_left);
        right = new MotorControllerGroup(front_right, back_right);
        dt = new DifferentialDrive(left, right);
        front_left.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        front_right.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        left.setInverted(false);
        right.setInverted(true);

        
    }

    public void set_speeds(double xSpeed, double zRotation){
        dt.arcadeDrive(xSpeed, zRotation);
    }

    public DifferentialDriveWheelSpeeds set_speeds_voltage(double leftSpeed, double rightSpeed, double leftStart, double rightStart){
        double leftV = feedforward.calculate(leftSpeed);
        double rightV = feedforward.calculate(rightSpeed);
        //leftV = leftPIDController.calculate(front_left.getSelectedSensorPosition()-leftStart, leftSpeed)+ leftV;
        //rightV = rightPIDController.calculate(front_right.getSelectedSensorPosition()-rightStart, rightSpeed)+ rightV;
        System.out.println("setvoltage function - left velocity: " + leftV + ", right velocity: " + rightV);

        //left.setVoltage(leftV);
        //right.setVoltage(rightV);
        return new DifferentialDriveWheelSpeeds(leftV, rightV);
        // front_left.feed();
        // front_right.feed();
        // back_left.feed();
        // back_right.feed();
    }

    public void stop(){
        dt.arcadeDrive(0, 0);
    }

    //KINEMATICS begin here
    
    public AHRS navx = new AHRS();
    public DifferentialDriveKinematics kin = new DifferentialDriveKinematics(Units.inchesToMeters(Distances.track_width));
    public DifferentialDriveOdometry odom = new DifferentialDriveOdometry(getHeading());
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Distances.kS, Distances.kV, Distances.kA);
    private PIDController leftPIDController = new PIDController(Distances.kp, Distances.ki, Distances.kd);
    private PIDController rightPIDController = new PIDController(Distances.kp, Distances.ki, Distances.kd);

    public Rotation2d getHeading(){
        return Rotation2d.fromDegrees(navx.getAngle());
    }
    public double convertMeters(double sensorCounts){
        double motorRotations = (double)sensorCounts / Distances.revs_count;
        double wheelRotations = motorRotations / Distances.gear_ratio;
        double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(Distances.wheel_size));
        return positionMeters;
    }

    public double convertMeters2(double sensorCount){
        return Units.inchesToMeters(sensorCount/Distances.encoder_ratio);
    }

    public SimpleMotorFeedforward getFeedforward(){
        return feedforward;
    }

    public PIDController getLeftPIDController(){
        return leftPIDController;
    }

    public PIDController getRightPIDController(){
        return rightPIDController;
    }

    public double getVel(){
        double mps = convertMeters(front_left.getSelectedSensorVelocity()) * 10.0;
        return mps;
    }

    public double getVel2(){
        double mps = Units.inchesToMeters(front_left.getSelectedSensorVelocity()/Distances.encoder_ratio)*10;
        return mps;
    }


    // public DifferentialDriveWheelSpeeds getSpeeds(){
        
    //     return new DifferentialDriveWheelSpeeds(Units.inchesToMeters(front_left.getSelectedSensorVelocity() * Distances.encoder_ratio)/60, Units.inchesToMeters(front_right.getSelectedSensorVelocity() * Distances.encoder_ratio)/60);
    // }
}
