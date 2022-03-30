package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Constants.Speeds;

public class Shooter {
    public WPI_TalonSRX shooter;

    public Shooter(int port){
        shooter = new WPI_TalonSRX(port);
        shooter.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    }

    public void set_speed(double speed){
        shooter.set(speed*-1);
    }

    public void set_voltage(double voltage){
        shooter.setVoltage(voltage);
    }

    public double get_velocity(){
        return shooter.getSelectedSensorVelocity();
    }

    public double get_pos(){
        return shooter.getSelectedSensorPosition();
    }

    public boolean is_ready(double vel){
        if(get_velocity()>vel){
            return true;
        }
        return false;
    }

    public double get_vel_threshold(double voltage, boolean front){
        if(front){
            return Speeds.vel_threshold*(2837.05*voltage-1140.09);//tested equation for front shooter
        }
        return Speeds.vel_threshold*(3058.85*voltage-2225.71);//tested equation for back shooter
    }
    public void stop(){
        shooter.set(0);
    }
}
