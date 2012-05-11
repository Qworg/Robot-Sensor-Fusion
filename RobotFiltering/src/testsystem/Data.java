package testsystem;

import java.io.*;
import java.util.Scanner;
import java.util.Vector;

import javax.vecmath.Matrix3d;
import javax.vecmath.Matrix4d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

//A class for encompassing all data storage from the file.

public class Data {
	private Vector<IMUEntry> IMUData = new Vector<IMUEntry>();

	private Vector<GPSEntry> GPSData = new Vector<GPSEntry>();

	private Vector<OdoEntry> OdoData = new Vector<OdoEntry>();
	
	private Vector<CompassEntry> CompassData = new Vector<CompassEntry>();

	private int EntryNum = 0;

	public Data(String dataFileName) {
		if (dataFileName != null) {
			EntryNum = 0;
			try {
				// Open the file that is the first
				// command line parameter
				System.out.println("Attempting to open " + dataFileName);
				FileReader fstream = new FileReader(dataFileName);

				// Convert our input stream to a DataInputStream
				BufferedReader in = new BufferedReader(fstream);

				System.out.println("File Open!" + dataFileName);
				// Continue to read lines while
				// there are still some left to read
				// Blow past the header
				in.readLine();
				String line = "";
				Double timestamp = 0.0;
				Double elapsedtime = 0.0;
				Vector<Double> IMUAccel = new Vector<Double>();
				Vector<Double> IMURAccel = new Vector<Double>();
				Vector<Double> IMURate = new Vector<Double>();
				Double X, Y, Theta;
				Double Heading;
				Double GPSNorthY, GPSEastX;

				while ((line = in.readLine()) != null) {
					// pull down a line, then crush it
//					System.out.println("Reading a Line!");
					Scanner s = new Scanner(line);
//					System.out.println(line);
					timestamp = Double.valueOf(s.next());
					elapsedtime = Double.valueOf(s.next());
					IMUAccel.clear();
					IMUAccel.add(Double.valueOf(s.next()));
					IMUAccel.add(Double.valueOf(s.next()));
					IMUAccel.add(Double.valueOf(s.next()));
					IMURAccel.clear();
					//These aren't really angles.  Fake it!
					IMURAccel.add(Double.valueOf(s.next()));
					IMURAccel.add(Double.valueOf(s.next()));
					IMURAccel.add(0.0);
					Heading = Double.valueOf(s.next());
					IMURate.clear();
					IMURate.add(Double.valueOf(s.next()));
					IMURate.add(Double.valueOf(s.next()));
					IMURate.add(Double.valueOf(s.next()));
					X = Double.valueOf(s.next());
					Y = Double.valueOf(s.next());
					Theta = Double.valueOf(s.next());
					GPSNorthY = Double.valueOf(s.next());
					GPSEastX = Double.valueOf(s.next());
					//System.out.println(X + "," + Y + "," + Theta);
					IMUData.add(new IMUEntry(IMUAccel, IMURAccel, timestamp));
					GPSData.add(new GPSEntry(GPSEastX, GPSNorthY, timestamp));
					OdoData.add(new OdoEntry(X, Y, Theta, timestamp));
					CompassData.add(new CompassEntry(Heading, timestamp));
					EntryNum++;
				}

				in.close();
			} catch (Exception e) {
				System.err.println("File input error");
			}
		} else
			System.out.println("Invalid parameters");
	}

	public IMUEntry GetIMUEntry(int iEntry) {
		if (IMUData != null)
			return IMUData.elementAt(iEntry);
		else
			System.out.println("IMUDaaaata is empty?");
			return null;
	}
	
	public Vector<Double> GetIMUVelocity(int iEntry, Vector<Double> DeltaH) {
		// IMU Data
		// IMU Time
		// IMU X+ = backwards
		// IMU Y+ = moves left (viewed from behind)
		// IMU Z+ = moves down
		// IMU XRot = RHR around backwards facing axis (robot rotates
		// counterclockwise roll as viewed from behind)
		// IMU YRot = RHR around left axis (Pitch towards ground = higher
		// number)
		// IMU ZRot = RHR around down axis (Robot turning right = higher number)
		Vector<Double> tempAcc = IMUData.get(iEntry).getIMUAcceleration();
		Double TS = IMUData.get(iEntry).getIMUTimestamp();
		
		//System.out.println("IMU Accels: " + tempAcc.get(0) + "," + tempAcc.get(1) + "," + tempAcc.get(2));
		
		Matrix4d mRotX = new Matrix4d();
		mRotX.rotX(DeltaH.get(0));
		Matrix4d mRotY = new Matrix4d();
		mRotY.rotY(DeltaH.get(1));
		Matrix4d mRotZ = new Matrix4d();
		mRotZ.rotZ(-DeltaH.get(2));
		
		mRotX.mul(mRotY);
		mRotX.mul(mRotZ);
		
		Point3d Accel = new Point3d();
		Accel.set(tempAcc.get(0), tempAcc.get(1), tempAcc.get(2));
		
		mRotX.transform(Accel);
		
		tempAcc.set(0, Accel.x);
		tempAcc.set(1, Accel.y);
		tempAcc.set(2, Accel.z);
		//System.out.println("IMU After Rotation Accels: " + tempAcc.get(0) + "," + tempAcc.get(1) + "," + tempAcc.get(2));
		
		return tempAcc;
	}
	
	public Vector<Double> GetIMURotVelocity(int iEntry){
		Vector <Double> tempRot = IMUData.get(iEntry).getIMURotAccel();
		Double TS = IMUData.get(iEntry).getIMUTimestamp();
		
		System.out.println("Don't call this!  Broken!");
		tempRot.set(0, tempRot.get(0)*TS);
		tempRot.set(0, tempRot.get(1)*TS);
		tempRot.set(0, tempRot.get(2)*TS);
		return tempRot;	
	}
	
	public OdoEntry GetOdoEntry(int iEntry) {
		if (OdoData != null)
			return OdoData.elementAt(iEntry);
		else
			return null;
	}
	public GPSEntry GetGPSEntry(int iEntry) {
		if (GPSData != null)
			return GPSData.elementAt(iEntry);
		else
			return null;
	}
	public CompassEntry GetCompassEntry(int iEntry){
		if (CompassData != null)
			return CompassData.elementAt(iEntry);
		else
			return null;
	}
	public int getLength(){
		return EntryNum;
	}
	
}
