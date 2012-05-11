package filters;

import java.util.Date;
import java.util.Vector;

import testsystem.CompassEntry;
import testsystem.Data;
import testsystem.GPSEntry;
import testsystem.IMUEntry;
import testsystem.OdoEntry;
import Jama.Matrix;
import controllers.*;
import java.io.*;

public class EKF implements Filter {

	/**
	 * @param args
	 */
	private boolean ProcessOn = false;

	private boolean ProcessDone = false;

	private Date TimeOn, TimeOff;

	private Long TimeElapsed;

	private Data dDataset;

	private EKFControl cController;
	
	private Vector<GPSEntry> Output = new Vector<GPSEntry>();
	
	private Vector<Matrix> Resid = new Vector<Matrix>();
	
	private Double initQ, initR;

	public EKF(Control controller, Data Dataset, Double defQ, Double defR) {
		cController = (EKFControl) controller;
		dDataset = Dataset;
		initQ = defQ;
		initR = defR;
	}

	public Boolean finishedProcess() {
		return ProcessDone;
	}

	public void outputData(String FileName) {
		try {
			BufferedWriter out = new BufferedWriter(new FileWriter(FileName+TimeElapsed+".txt"));
			int len = Output.size();
			System.out.println("Time Elapsed: "+TimeElapsed);
			GPSEntry A;
			Matrix B;
			out.write("#Timestep NorthX EastY ResidX ResidY ResidZ");
			out.newLine();
			for (int i = 0; i < len; i++) {
				A = Output.elementAt(i);
				B = Resid.elementAt(i);
				out.write(A.getGPSTimestamp() + " " + A.getEastX() + " " + A.getNorthY() + " " + B.get(0, 0) + " "+ B.get(0, 1) + " "+ B.get(0, 2));
				out.newLine();
			}
			out.close();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

	}

	public void setControl(Control Controller) {
		if (!ProcessOn) {
			// Change the Controller
			cController = (EKFControl) Controller;
		} else
			System.out.println("Can't reset controller in mid-process!");
	}

	public void setData(Data Dataset) {
		if (!ProcessOn) {
			// Change the Dataset
			dDataset = Dataset;
		} else
			System.out.println("Can't reset data in mid-process!");
	}

	public void startProcess() {
		// Start the process!
		ProcessOn = true;
		TimeOn = new Date();
		
		
		// Get the length of the dataset
		int iLength = dDataset.getLength();
		GPSEntry gGPS;
		IMUEntry iIMU;
		OdoEntry oOdo;
		CompassEntry cComp;
		Double dTS;
		Vector<Double> vIMU = new Vector<Double>();
		Vector<Double> vComp = new Vector<Double>();
		Vector<Double> vOdoR = new Vector<Double>();
		Vector<Double> vOdo = new Vector<Double>();
		Vector<Double> vGPS = new Vector<Double>();
		Vector<Double> vOldDS = new Vector<Double>();
		
		//Build the First States!
		Vector<Vector<Double>> ZeroHeadingState = new Vector<Vector<Double>>();
		Vector<Vector<Double>> HeadingState = new Vector<Vector<Double>>();
		Vector<Vector<Double>> DriveState = new Vector<Vector<Double>>();
		Vector<Vector<Double>> PoseState = new Vector<Vector<Double>>();
		Vector<Double> TempVec = new Vector<Double>();
		//X,Y,Z
		//XMinus, YMinus, ZMinus
		TempVec.add(0.0);
		TempVec.add(0.0);
		TempVec.add(0.0);
		DriveState.add(TempVec);
		DriveState.add(TempVec);
		TempVec.set(2, dDataset.GetCompassEntry(0).GetRad());
		HeadingState.add(TempVec);
		HeadingState.add(TempVec);
		TempVec.set(2, dDataset.GetOdoEntry(0).getOdoTheta());
		ZeroHeadingState.add(TempVec);
		ZeroHeadingState.add(TempVec);
		TempVec.clear();
		TempVec.add(dDataset.GetGPSEntry(0).getEastX());
		TempVec.add(dDataset.GetGPSEntry(0).getNorthY());
		TempVec.add(0.0);
		PoseState.add(TempVec);
		PoseState.add(TempVec);
		//PX,PY,PZ
		TempVec.clear();
		TempVec.add(0.0);
		TempVec.add(0.0);
		TempVec.add(0.0);
		ZeroHeadingState.add(TempVec);
		HeadingState.add(TempVec);
		DriveState.add(TempVec);
		PoseState.add(TempVec);
		//Qx, Qy, Qz
		TempVec.clear();
		TempVec.add(initQ);
		TempVec.add(initQ);
		TempVec.add(initQ);
		ZeroHeadingState.add(TempVec);
		HeadingState.add(TempVec);
		DriveState.add(TempVec);
		PoseState.add(TempVec);
		//Rx, Ry, Rz
		TempVec.clear();
		TempVec.add(initR);
		TempVec.add(initR);
		TempVec.add(initR);
		ZeroHeadingState.add(TempVec);
		HeadingState.add(TempVec);
		DriveState.add(TempVec);
		//Rx, Ry, Rz
		TempVec.clear();
		TempVec.add(2.5);
		TempVec.add(2.5);
		TempVec.add(2.5);
		PoseState.add(TempVec);
		//Ax, Ay, Az
		TempVec.clear();
		TempVec.add(1.0);
		TempVec.add(1.0);
		TempVec.add(1.0);
		ZeroHeadingState.add(TempVec);
		HeadingState.add(TempVec);
		DriveState.add(TempVec);
		PoseState.add(TempVec);
		
		Double OldOdoTheta = 0.0;
		Double OldOdoX = 0.0;
		Double OldOdoY = 0.0;
		// Start the roll!
		for (int i = 0; i < iLength; i++) {
			// Get the data @ i
			gGPS = dDataset.GetGPSEntry(i);
			iIMU = dDataset.GetIMUEntry(i);
			oOdo = dDataset.GetOdoEntry(i);
			dTS = gGPS.getGPSTimestamp();
			//vRIMU = dDataset.GetIMURotVelocity(i);
			cComp = dDataset.GetCompassEntry(i);
			
			//Ticker
			if (i%1000 == 0)
				System.out.print(".");
			
			// Cascading EKFs!
			// Kalman Filtering Section

			// Step 1: Cut a hole in a box
			// Step 2: Put the junk in the box
			// Step 3: Get them to open the box

			// Actually:
			// Step 1: KF ODO Theta as the drive and Compass as the Measure
			// Step 2: Use the combined heading to lineralize the IMU acceleration
			// Step 3: KF linearlized accel as a drive and ODO X and Y as the Measure
			// Step 4: KF Combined as a drive and GPS X and Y as the Measure
			//Step 0: KF the proper order again...
			vIMU.clear();
			vIMU.add(0.0);
			vIMU.add(0.0);
			vIMU.add(iIMU.getIMURotAccel().get(2));
			vOdoR.clear();
			vOdoR.add(0.0);
			vOdoR.add(0.0);
			vOdoR.add((oOdo.getOdoTheta()-OldOdoTheta));
			OldOdoTheta = oOdo.getOdoTheta();
			ZeroHeadingState = KStep(ZeroHeadingState, vOdoR, vIMU, dTS);
			//Step 1:
			vOdoR.clear();
			vOdoR = ZeroHeadingState.get(0);
			vOdoR.set(0, vOdoR.get(0)/dTS);
			vOdoR.set(1, vOdoR.get(1)/dTS);
			vOdoR.set(2, vOdoR.get(2)/dTS);
			vComp.clear();
			vComp.add(0.0);
			vComp.add(0.0);
			vComp.add(cComp.GetRad());
			HeadingState = KStep(HeadingState, vComp, vOdoR, dTS);
			//Step 2:
			vIMU = dDataset.GetIMUVelocity(i, HeadingState.get(0));
			//Step 3:
			vOdo.clear();
			vOdo.add(oOdo.getOdoX()-OldOdoX);
			vOdo.add(oOdo.getOdoY()-OldOdoY);
			vOdo.add(0.0);
			OldOdoX = oOdo.getOdoX();
			OldOdoY = oOdo.getOdoY();
			DriveState = KStep(DriveState, vOdo, vIMU, dTS);
			//Step 4:
			vOdo.clear();
			vOdo.add(DriveState.get(0).get(0)/dTS);
			vOdo.add(DriveState.get(0).get(1)/dTS);
			vOdo.add(DriveState.get(0).get(2)/dTS);
			vGPS.clear();
			vGPS.add(gGPS.getEastX());
			vGPS.add(gGPS.getNorthY());
			vGPS.add(0.0);
			PoseState = KStep(PoseState, vGPS, vOdo, dTS);
			
			Output.add(new GPSEntry(PoseState.elementAt(0).elementAt(0), PoseState.elementAt(0).elementAt(1), dTS));
			Matrix tempResid = new Matrix(1,3,0.0);
			tempResid.set(0, 0, PoseState.get(0).get(0)-PoseState.get(1).get(0));
			tempResid.set(0, 1, PoseState.get(0).get(1)-PoseState.get(1).get(1));
			tempResid.set(0, 2, PoseState.get(0).get(2)-PoseState.get(1).get(2));
			Resid.add(tempResid);
		}

		// Process done!
		TimeOff = new Date();
		TimeElapsed = TimeOff.getTime() - TimeOn.getTime();
		ProcessOn = false;
		ProcessDone = true;

	}

	private Vector<Vector<Double>> KStep(Vector<Vector<Double>> vOldState, Vector<Double> dMeasurement, Vector<Double> dStateUpdate, Double dTimeStep) {
		// Big Kalman - individual dR and dQ
		Vector<Vector<Double>> vOutput = new Vector<Vector<Double>>();
		
		// Extract all of the necessary variables
		Double XKMinus = vOldState.elementAt(0).elementAt(0);
		Double YKMinus = vOldState.elementAt(0).elementAt(1);
		Double ZKMinus = vOldState.elementAt(0).elementAt(2);
		Double XPkminus = vOldState.elementAt(2).elementAt(0);
		Double YPkminus = vOldState.elementAt(2).elementAt(1);
		Double ZPkminus = vOldState.elementAt(2).elementAt(2);
		Double dQx = vOldState.elementAt(3).elementAt(0);
		Double dQy = vOldState.elementAt(3).elementAt(1);
		Double dQz = vOldState.elementAt(3).elementAt(2);
		Double dRx = vOldState.elementAt(4).elementAt(0);
		Double dRy = vOldState.elementAt(4).elementAt(1);
		Double dRz = vOldState.elementAt(4).elementAt(2);
		Double alphax = vOldState.elementAt(5).elementAt(0);
		Double alphay = vOldState.elementAt(5).elementAt(1);
		Double alphaz = vOldState.elementAt(5).elementAt(2);

		// Project the New State ahead
		Double XMinus = XKMinus + dStateUpdate.elementAt(0) * dTimeStep;
		Double YMinus = YKMinus + dStateUpdate.elementAt(1) * dTimeStep;
		Double ZMinus = ZKMinus + dStateUpdate.elementAt(2) * dTimeStep;

		// Project the Error Covariance Ahead
		Double XPMinus = alphax * alphax * (XPkminus * alphax * alphax) + dQx;
		Double YPMinus = alphay * alphay * (YPkminus * alphay * alphay) + dQy;
		Double ZPMinus = alphaz * alphaz * (ZPkminus * alphaz * alphaz) + dQz;

		// Compute Kalman Gain
		Double XKal = XPMinus / (XPMinus + dRx / (alphax * alphax));
		Double YKal = YPMinus / (YPMinus + dRy / (alphay * alphay));
		Double ZKal = ZPMinus / (ZPMinus + dRz / (alphaz * alphaz));

		// Update Estimate with measurement
		Double X = XMinus + (XKal * (dMeasurement.elementAt(0) - XMinus));
		Double Y = YMinus + (YKal * (dMeasurement.elementAt(1) - YMinus));
		Double Z = ZMinus + (ZKal * (dMeasurement.elementAt(2) - ZMinus));

		// Update the Error Covariance
		Double PX = (1 - XKal) / XPMinus;
		Double PY = (1 - YKal) / YPMinus;
		Double PZ = (1 - ZKal) / ZPMinus;

		// Error Checking Prints for Debug
		if (X.isNaN()) {
			System.out.println("X is Not A Number!  Freakout Time! Output: " + X + "," + Y + "," + Z);
			System.out.print("\007");
			System.out.print("\007");
			System.out.print("\007");
			System.out.print("\007");
			System.out.flush();
			try {
				Thread.sleep(10000);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		if (Y.isNaN()){
			System.out.println("Y is Not A Number!  Freakout Time! Output: " + X + "," + Y + "," + Z);
			System.out.print("\007");
			System.out.print("\007");
			System.out.print("\007");
			System.out.print("\007");
			System.out.flush();
			try {
				Thread.sleep(10000);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		if (Z.isNaN()) {
			System.out.println("Z is Not A Number!  Freakout Time! Output: " + X + "," + Y + "," + Z);
			System.out.println("Minus " + XMinus+ "," +YMinus+ "," +ZMinus);
			System.out.println("PMinus " + XPMinus+ "," +YPMinus+ "," +ZPMinus);
			System.out.println("Kal "+ XKal+ "," +YKal+ "," +ZKal);
			System.out.println("P " + PX+ "," +PY+ "," +PZ);
			System.out.println("Q "+dQx+ "," +dQy+ "," +dQz);
			System.out.println("R "+dRx+ "," +dRy+ "," +dRz);
			System.out.print("\007");
			System.out.print("\007");
			System.out.print("\007");
			System.out.print("\007");
			System.out.flush();
			try {
				Thread.sleep(10000);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		
		// Output
		Vector<Double> RowZero = new Vector<Double>();
		RowZero.add(X);
		RowZero.add(Y);
		RowZero.add(Z);
		//NOT OUTPUT... =0
		Vector<Double> RowOne = new Vector<Double>();
		RowOne.add(XMinus);
		RowOne.add(YMinus);
		RowOne.add(ZMinus);
		Vector<Double> RowTwo = new Vector<Double>();
		RowTwo.add(PX);
		RowTwo.add(PY);
		RowTwo.add(PZ);
		
		vOutput.add(RowZero);
		vOutput.add(RowOne);
		vOutput.add(RowTwo);
		vOutput.add(vOldState.elementAt(3));
		vOutput.add(vOldState.elementAt(4));
		vOutput.add(vOldState.elementAt(5));
		
		// Update alpha and R/Q
		cController.inputState(vOutput);

		cController.startProcess();
		while(!cController.finishedProcess()){
			try {
				Thread.sleep(10);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
		vOutput.remove(3);
		vOutput.remove(3);
		vOutput.remove(3);
		vOutput.add(cController.getQ());
		vOutput.add(cController.getR());
		vOutput.add(cController.getAlpha());
		
		
		return vOutput;
	}

	public long getTimeElapsed() {
		return TimeElapsed;
	}
	public Vector<GPSEntry> getOutput() {
		if (ProcessDone){
			return Output;
		}
		return null;
	}

	public Vector<Matrix> getResiduals() {
		if (ProcessDone){
			return Resid;
		}
		return null;
	}

}
