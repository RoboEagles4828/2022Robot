package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.Constants.Distances;

import java.util.Arrays;

import com.kauailabs.navx.frc.AHRS;

public class Limelight {

    //main network table
    public NetworkTable table;
    private NetworkTableEntry tx;
    private NetworkTableEntry ty;
    private NetworkTableEntry ta;
    private NetworkTableEntry tv;

    //distance stuff
    private double mountAngle;

    private double prevError;
    private double sumError;

    private RollingAvg txAvg = new RollingAvg();

    private AHRS navx = new AHRS();

    private double[] pidConstants;

    public Limelight(NetworkTable table) {
        this.table=table;
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        tv = table.getEntry("tv");
        this.prevError = 0;
        this.sumError = 0;
        this.mountAngle = 30;
    }

    public Limelight(NetworkTable table, double mountAngle) {
        this(table);
        this.mountAngle = mountAngle;
    }

    public Limelight(NetworkTable table, double[] pidConstants) {
        this(table);
        this.pidConstants = pidConstants;
    }

    public double getX() {
        return tx.getDouble(0);
    }

    public double getY() {
        return ty.getDouble(0);
    }

    public double getAngle() {
        return ta.getDouble(0);
    }

    public boolean hasValidTarget() {
        return tv.getDouble(0) == 1 ? true : false;
    }

    public void setLEDState(int state) {
        table.getEntry("ledMode").setNumber(state);
    }

    public double getDistanceFromTarget(double goalHeight, double camHeight) {
        double distance = 0;
        if (!hasValidTarget()) {
            distance = -1;
        }
        else {
            distance = (goalHeight - camHeight) / Math.tan((mountAngle + ty.getDouble(0)) * (Math.PI / 180));
        }
        return distance;
    }

    private class RollingAvg {
        private int size;
        private double total = 0;
        private int index = 0;
        private double samples[];
    
        public RollingAvg(int size) {
            this.size = size;
            samples = new double[size];
            Arrays.fill(samples, 0);
        }
    
        public RollingAvg() {
            this(4);
        }
    
        public void add(double x) {
            total -= samples[index];
            samples[index] = x;
            total += x;
            if (++index == size) {
                index = 0;
            }
        }
    
        public double getAverage() {
            return total / size;
        }
    }

    //-right +left
    public double getAlignmentAdjustmentPID() {
        txAvg.add(tx.getDouble(0));
        double error = txAvg.getAverage() / 27.0;

        double adjustment = 0;

        if (hasValidTarget() && Math.abs(error) < Distances.limelight_threshold) {
            adjustment = 0;
        }
        else {
            adjustment = centerAlgorithm(error, pidConstants[0], pidConstants[1], pidConstants[2]);
        }
        return adjustment;
    }

    public double getAlignementAdjustmentSimple(double speed, double threshold) {
        double x = tx.getDouble(0);
        double adjustment = 0;
        if (hasValidTarget()) {
            if (x > threshold && x < -threshold) adjustment = 0;
            else if (x > threshold) adjustment = -speed;
            else if (x < threshold) adjustment = speed;
            else adjustment = 0;
        }

        return adjustment;
    }

    public double getAlignmentAdjustmentOld() {
        double min = 0.1;
        double adjustment = 0;
        double kP = 0.1;
        double x = tx.getDouble(0);
        double headingErr = -x;
        if (hasValidTarget()) {
            if (x > 1) {
                adjustment = kP * headingErr - min;
            }
            else if (x < 1) {
                adjustment = kP * headingErr + min;
            }
        }
        return adjustment;
    }

    private double centerAlgorithm(double error, double kP, double kI, double kD) {
        double diffError = (error - prevError);

        sumError += error;
        double output = (kP * error) + (kI * sumError) + (kD * diffError);

        if (output > 0 && output < 0.05) {
            output = 0.05;
        }
        else if (output < 0 && output > -0.05) {
            output = -0.05;
        }
        else if (output > 0.5) {
            output = 0.5;
        }
        else if (output < -0.5) {
            output = -0.5;
        } 

        prevError = error;

        return output;
    }

    public double getAlignmentAdjustmentNavX(double speed) {
        double x = tx.getDouble(0);

        double adjustment = 0;

        if (x < 1) {
            if (navx.getAngle() % 360 < x - Distances.turning_error) {
                adjustment = speed;
            }
        }
        else if (x > 1) {
            if (navx.getAngle() % 360 > -x + Distances.turning_error) {
                adjustment = -speed;
            }
        }

        return adjustment;
    }
}