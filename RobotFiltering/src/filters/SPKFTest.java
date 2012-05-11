package filters;

import java.util.Date;
import java.util.Random;
import java.util.Vector;
import Jama.*;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.lang.Math;

import testsystem.CompassEntry;
import testsystem.Data;
import testsystem.GPSEntry;
import testsystem.IMUEntry;
import testsystem.OdoEntry;
import controllers.*;

public class SPKFTest implements Filter {

	/**
	 * @param args
	 */

	private boolean ProcessOn = false;

	private boolean ProcessDone = false;

	private Date TimeOn, TimeOff;

	private Long TimeElapsed;

	private Data dDataset;

	private Double dQ, dR;

	protected SPKFControl cController;

	private Vector<GPSEntry> Output = new Vector<GPSEntry>();

	private Vector<Matrix> Resid = new Vector<Matrix>();

	private Vector<CompassEntry> OutputH = new Vector<CompassEntry>();

	private Vector<Matrix> ResidH = new Vector<Matrix>();

	private Vector<CompassEntry> OutputH1 = new Vector<CompassEntry>();

	private Vector<Matrix> ResidH1 = new Vector<Matrix>();

	private Vector<OdoEntry> OutputD = new Vector<OdoEntry>();

	private Vector<Matrix> ResidD = new Vector<Matrix>();

	private Double defA, defB, defK;

	// Specific Variables
	protected static int L = 9; // L is the Dimension of the States (X, Y, Z),

	// plus the dimension of the two errors

	public SPKFTest(Control Controller, Data Dataset, Double initQ, Double initR) {
		cController = (SPKFControl) Controller;
		dDataset = Dataset;
		this.setAlpha(0.02);
		this.setBeta(2.0);
		this.setKappa(0.0);
		cController.setL(L);
		dQ = initQ;
		dR = initR;
	}

	public SPKFTest(Control Controller, Data Dataset, Double initQ, Double initR, Double Alpha, Double Beta, Double Kappa) {
		cController = (SPKFControl) Controller;
		dDataset = Dataset;
		this.setAlpha(Alpha);
		this.setBeta(Beta);
		this.setKappa(Kappa);
		cController.setL(L);
		dQ = initQ;
		dR = initR;
	}

	public Boolean finishedProcess() {
		return ProcessDone;
	}

	public void outputData(String FileName) {
		try {
			BufferedWriter out = new BufferedWriter(new FileWriter(FileName));
			BufferedWriter outH1 = new BufferedWriter(new FileWriter(FileName + "HeadingOne"));
			BufferedWriter outH = new BufferedWriter(new FileWriter(FileName + "Heading"));
			BufferedWriter outD = new BufferedWriter(new FileWriter(FileName + "Drive"));
			int len = Output.size();
			System.out.println("Len = " + len);
			GPSEntry A;
			Matrix B;
			out.write("#Timestep EastX NorthY ResidX ResidY ResidZ");
			out.newLine();
			for (int i = 0; i < len; i++) {
				A = Output.elementAt(i);
				B = Resid.elementAt(i);
				out.write(A.getGPSTimestamp() + " " + A.getEastX() + " " + A.getNorthY() + " " + B.get(0, 0) + " " + B.get(1, 0) + " " + B.get(2, 0));
				// out.write(A.getGPSTimestamp() + "," + A.getEastX() + "," +
				// A.getNorthY() + "," + B.get(0, 0) + "," + B.get(0, 1) + "," +
				// B.get(0, 2));
				out.newLine();
			}
			out.close();
			CompassEntry C;
			outH1.write("Timestep, HeadingRads, ResidX, ResidY, ResidZ");
			outH1.newLine();
			for (int i = 0; i < len; i++) {
				C = OutputH1.elementAt(i);
				B = ResidH1.elementAt(i);
				outH1.write(C.GetTimestep() + "," + C.GetRad() + "," + B.get(0, 0) + "," + B.get(1, 0) + "," + B.get(2, 0));
				outH1.newLine();
			}
			outH1.close();
			outH.write("Timestep, HeadingRads, ResidX, ResidY, ResidZ");
			outH.newLine();
			for (int i = 0; i < len; i++) {
				C = OutputH.elementAt(i);
				B = ResidH.elementAt(i);
				outH.write(C.GetTimestep() + "," + C.GetRad() + "," + B.get(0, 0) + "," + B.get(1, 0) + "," + B.get(2, 0));
				outH.newLine();
			}
			outH.close();
			OdoEntry D;
			outD.write("Timestep, DriveX, DriveY, DriveDelta, ResidX, ResidY, ResidZ");
			outD.newLine();
			for (int i = 0; i < len; i++) {
				D = OutputD.elementAt(i);
				B = ResidD.elementAt(i);
				outD.write(D.getOdoTimestamp() + "," + D.getOdoX() + "," + D.getOdoY() + "," + D.getOdoTheta() + "," + B.get(0, 0) + "," + B.get(1, 0) + "," + B.get(2, 0));
				outD.newLine();
			}
			outD.close();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

	}

	public void setAlpha(Double Alpha) {
		if (!ProcessOn) {
			// Change Alpha
			cController.setAlpha(Alpha);
			defA = Alpha;
		} else
			System.out.println("Can't reset Alpha in mid-process!");
	}

	public void setBeta(Double Beta) {
		if (!ProcessOn) {
			// Change Beta
			cController.setBeta(Beta);
			defB = Beta;
		} else
			System.out.println("Can't reset Beta in mid-process!");
	}

	public void setKappa(Double Kappa) {
		if (!ProcessOn) {
			// Change Kappa
			cController.setKappa(Kappa);
			defK = Kappa;
		} else
			System.out.println("Can't reset Kappa in mid-process!");
	}

	public void setControl(Control Controller) {
		if (!ProcessOn) {
			// Change the Controller
			cController = (SPKFControl) Controller;
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
		// Generate new Variables
		GPSEntry gGPS;
		IMUEntry iIMU;
		OdoEntry oOdo;
		CompassEntry cComp;
		Double dTS;
		Vector<Double> vIMU = new Vector<Double>();
		Vector<Double> vComp = new Vector<Double>();
		Vector<Double> DeltaH = new Vector<Double>();
		Vector<Double> tempOdo = new Vector<Double>();
		Vector<Double> vGPS = new Vector<Double>();
		Vector<Double> vGreekHeading = new Vector<Double>();
		Vector<Double> vGreekHeading1 = new Vector<Double>();
		// Stores Alpha,
		// Beta and
		// Kappa for
		// each type of
		// controller
		Vector<Double> vGreekDrive = new Vector<Double>(); // Stores Alpha,
		// Beta and Kappa
		// for each type of
		// controller
		Vector<Double> vGreekPose = new Vector<Double>(); // Stores Alpha,
		// Beta and Kappa
		// for each type of
		// controller

		// Start the process!
		ProcessOn = true;
		TimeOn = new Date();
		// Get the length of the dataset
		int iLength = dDataset.getLength();

		// Build the First States
		vGreekHeading.add(defA);
		vGreekHeading.add(defB);
		vGreekHeading.add(defK);
		vGreekDrive = vGreekPose = vGreekHeading1 = vGreekHeading;

		// Matrix HeadingX = new Matrix(3, 3, 0.0);
		Matrix HeadingX = new Matrix(9, 1, 0.0);
		Matrix HeadingP = new Matrix(9, 9, 0.0);
		// Matrix Heading1X = new Matrix(3, 3, 0.0);
		Matrix Heading1X = new Matrix(9, 1, 0.0);
		Matrix Heading1P = new Matrix(9, 9, 0.0);
		// Matrix DriveX = new Matrix(3, 3, 0.0);
		Matrix DriveX = new Matrix(9, 1, 0.0);
		Matrix DriveP = new Matrix(9, 9, 0.0);
		// Matrix PoseX = new Matrix(3, 3, 0.0);
		Matrix PoseX = new Matrix(9, 1, 0.0);
		Matrix PoseP = new Matrix(9, 9, 0.0);

		// Reset the P values to norm
		HeadingP.set(3, 3, dQ);
		HeadingP.set(4, 4, dQ);
		HeadingP.set(5, 5, dQ);
		HeadingP.set(6, 6, dR);
		HeadingP.set(7, 7, dR);
		HeadingP.set(8, 8, dR);

		Heading1P.set(3, 3, dQ);
		Heading1P.set(4, 4, dQ);
		Heading1P.set(5, 5, dQ);
		Heading1P.set(6, 6, dR);
		Heading1P.set(7, 7, dR);
		Heading1P.set(8, 8, dR);

		DriveP.set(3, 3, dQ);
		DriveP.set(4, 4, dQ);
		DriveP.set(5, 5, dQ);
		DriveP.set(6, 6, dR);
		DriveP.set(7, 7, dR);
		DriveP.set(8, 8, dR);

		PoseP.set(3, 3, dQ);
		PoseP.set(4, 4, dQ);
		PoseP.set(5, 5, dQ);
		PoseP.set(6, 6, dR);
		PoseP.set(7, 7, dR);
		PoseP.set(8, 8, dR);

		gGPS = dDataset.GetGPSEntry(0);
		Heading1X.set(2, 0, dDataset.GetOdoEntry(0).getOdoTheta());
		HeadingX.set(2, 0, dDataset.GetCompassEntry(0).GetRad());
		DriveX.set(0, 0, dDataset.GetOdoEntry(0).getOdoX());
		DriveX.set(1, 0, dDataset.GetOdoEntry(0).getOdoY());
		PoseX.set(0, 0, gGPS.getEastX());
		PoseX.set(1, 0, gGPS.getNorthY());

		Double OldTheta = 0.0;
		Double OldGPSX = gGPS.getEastX();
		Double OldGPSY = gGPS.getNorthY();
		Double OldOdoX = 0.0;
		Double OldOdoY = 0.0;
		Double OldComp = dDataset.GetCompassEntry(0).GetRad();
		// Start the roll!
		for (int i = 0; i < iLength; i++) {
			// Get the data @ i
			gGPS = dDataset.GetGPSEntry(i);
			iIMU = dDataset.GetIMUEntry(i);
			oOdo = dDataset.GetOdoEntry(i);
			dTS = gGPS.getGPSTimestamp();
			cComp = dDataset.GetCompassEntry(i);
			// vRIMU = dDataset.GetIMURotVelocity(i);

			// Cascading SPKFs!
			// Sigma Point Kalman Filtering Section

			// Step 1: SPKF IMU Rotation (Acceleration in Robot) with Odo
			// Heading (Delta in World) to get ImpDeltaH
			// Step 2: SPKF ImpDelta (Delta Heading in World) with Heading in
			// World to get TrueHeading
			// Step 3: SPKF IMU Acceleration combined TrueHeading with Odo
			// (Delta Location in World) to get ImpDeltaL
			// Step 4: SPKF ImpDeltaL (Delta Location in World) with GPS
			// Location (Location in World) to get TrueLoc

			// Step 1:
			tempOdo.clear();
			tempOdo.add(0.0);
			tempOdo.add(0.0);
			tempOdo.add(oOdo.getOdoTheta() - OldTheta);
			OldTheta = oOdo.getOdoTheta();
			vIMU.clear();
			vIMU.add(0.0);
			vIMU.add(0.0);
			vIMU.add(iIMU.getIMURotAccel().get(2) * dTS);

			SPKFStep HeadingOne = new SPKFStep(Heading1P, Heading1X, vIMU, tempOdo, vGreekHeading1, dTS);
			while (!HeadingOne.StepFinished()) {
				try {
					Thread.sleep(10);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
			Heading1P = HeadingOne.getP();
			Heading1X = HeadingOne.getX();
			vGreekHeading1 = HeadingOne.getGreek();
			OutputH1.add(new CompassEntry(Heading1X.get(2, 0), dTS, true));
			ResidH1.add(HeadingOne.getResid());

			// Step 2:
			vComp.clear();
			vComp.add(0.0);
			vComp.add(0.0);
			vComp.add(cComp.GetRad());

			tempOdo.clear();
			tempOdo.add(Heading1X.get(0, 0));
			tempOdo.add(Heading1X.get(1, 0));
			tempOdo.add(Heading1X.get(2, 0));

			// System.out.println("Heading Before at Step " + i + ":");
			// HeadingP.print(3, 2);
			// HeadingX.print(3, 2);
			SPKFStep Heading = new SPKFStep(HeadingP, HeadingX, tempOdo, vComp, vGreekHeading, dTS);
			while (!Heading.StepFinished()) {
				try {
					Thread.sleep(10);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
			HeadingP = Heading.getP();
			HeadingX = Heading.getX();
			vGreekHeading = Heading.getGreek();
			OutputH.add(new CompassEntry(HeadingX.get(2, 0), dTS, true));
			ResidH.add(Heading.getResid());
			// System.out.println("Heading After at Step " + i + ":");
			// HeadingP.print(3, 2);
			// HeadingX.print(3, 2);
			DeltaH.clear();
			DeltaH.add(HeadingX.get(0, 0));
			DeltaH.add(HeadingX.get(1, 0));
			DeltaH.add(HeadingX.get(2, 0));

			// Step 3:
			vIMU = dDataset.GetIMUVelocity(i, DeltaH);
			for (int j = 0; j < vIMU.size(); j++) {
				// //DEBUG: What is vIMU?
				// System.out.println(vIMU.get(j));
				vIMU.set(j, (vIMU.get(j) * dTS));
				// //DEBUG: What is vIMU?
				// System.out.println(vIMU.get(j));
			}
			// This yields Velocity -^

			// Step 3:
			tempOdo.clear();
			tempOdo.add(oOdo.getOdoX() - OldOdoX);
			tempOdo.add(oOdo.getOdoY() - OldOdoY);
			tempOdo.add(0.0);
			OldOdoX = oOdo.getOdoX();
			OldOdoY = oOdo.getOdoY();
			// //
			// System.out.println("Drive Before at Step " + i + ":");
			// DriveP.print(3, 2);
			// DriveX.print(3, 2);
			SPKFStep Drive = new SPKFStep(DriveP, DriveX, vIMU, tempOdo, vGreekDrive, dTS);
			while (!Drive.StepFinished()) {
				try {
					Thread.sleep(10);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
			DriveP = Drive.getP();
			DriveX = Drive.getX();
			vGreekDrive = Drive.getGreek();
			OutputD.add(new OdoEntry(DriveX.get(0, 0), DriveX.get(1, 0), DeltaH.get(2), dTS));
			ResidD.add(Drive.getResid());
			// System.out.println("Drive After at Step " + i + ":");
			// DriveP.print(3, 2);
			// DriveX.print(3, 2);

			// Step 4:
			tempOdo.clear();
			tempOdo.add(DriveX.get(0, 0));
			tempOdo.add(DriveX.get(1, 0));
			tempOdo.add(DriveX.get(2, 0));
			vGPS.clear();
			vGPS.add(gGPS.getEastX());// -OldGPSX);
			vGPS.add(gGPS.getNorthY());// -OldGPSY);
			vGPS.add(0.0);
			OldGPSX = gGPS.getEastX();
			OldGPSY = gGPS.getNorthY();

			// System.out.println("Pose Before at Step " + i + ":");
			// PoseP.print(3, 2);
			// PoseX.print(3, 2);
			SPKFStep Pose = new SPKFStep(PoseP, PoseX, tempOdo, vGPS, vGreekPose, dTS);
			while (!Pose.StepFinished()) {
				try {
					Thread.sleep(10);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
			PoseP = Pose.getP();
			PoseX = Pose.getX();
			vGreekPose = Pose.getGreek();
			// System.out.println("GPS :" + gGPS.getEastX() + ", " +
			// gGPS.getNorthY());
			// System.out.println("Pose After at Step " + i + ":");
			// PoseP.print(3, 2);
			// PoseX.print(3, 2);
			GPSEntry GPSOut = new GPSEntry(PoseX.get(0, 0), PoseX.get(1, 0), dTS);
			// System.out.println(GPSOut.toString());
			Output.add(GPSOut);
			Resid.add(Pose.getResid());

			// System.out.print(".");
		}

		// Process done!
		TimeOff = new Date();
		TimeElapsed = TimeOff.getTime() - TimeOn.getTime();
		ProcessOn = false;
		ProcessDone = true;

	}

	public long getTimeElapsed() {
		return TimeElapsed;
	}

	private class SPKFStep {
		private Matrix P;

		private Matrix X;

		private Matrix Residual;

		private Double dTimestep;

		private Vector<Double> dControl;

		private Vector<Double> dMeasure;

		private Vector<Double> dGreek;

		private boolean StepDone = false;

		void setP(Matrix PDash) {
			P = PDash;
		}

		void setX(Matrix XDash) {
			X = XDash;
		}

		Matrix getP() {
			return P;
		}

		Matrix getX() {
			return X;
		}

		Matrix getResid() {
			return Residual;
		}

		Vector<Double> getGreek() {
			return dGreek;
		}

		boolean StepFinished() {
			return StepDone;
		}

		SPKFStep(Matrix PDash, Matrix XDash, Vector<Double> Control, Vector<Double> Measurement, Vector<Double> Greek, Double dInTimestep) {
			P = PDash;
			X = XDash;
			dGreek = Greek;
			dTimestep = dInTimestep;
			dControl = Control;
			dMeasure = Measurement;
			RunSPKF();
		}

		private Matrix SqrtSPKF(Matrix PDash) {
			// Only works with a symmetric Positive Definite matrix
			// [V,D] = eig(A)
			// S = V*diag(sqrt(diag(D)))*V'
			//			
			// //PDash in
			// System.out.println("PDash: ");
			// PDash.print(3, 2);

			// Matrix NegKeeper = Matrix.identity(9, 9);
			// //Testing Forced Compliance
			// for(int i = 0; i< 9; i++){
			// if (PDash.get(i, i)< 0){
			// NegKeeper.set(i,i,-1);
			// PDash.set(i, i, -PDash.get(i, i));
			// }
			// }
			EigenvalueDecomposition eig = PDash.eig();
			Matrix V = eig.getV();
			Matrix D = eig.getD();
			int iDSize = D.getRowDimension();
			for (int i = 0; i < iDSize; i++) {
				D.set(i, i, Math.sqrt(D.get(i, i)));
			}
			Matrix S = V.times(D).times(V.inverse());
			// S = S.times(NegKeeper);
			return S;
		}

		private void RunSPKF() {
			// SPKF Steps:
			// 1) Generate Test Points
			// 2) Propagate Test Points
			// 3) Compute Predicted Mean and Covariance
			// 4) Compute Measurements
			// 5) Compute Innovations and Cross Covariance
			// 6) Compute corrections and update

			// Create a random variable
			Random r = new Random();

			// Line up initial variables from the controller!
			Double dAlpha = dGreek.get(0);
			Double dBeta = dGreek.get(1);
			cController.setAlpha(dAlpha);
			cController.setBeta(dBeta);
			cController.setKappa(dGreek.get(2));
			Double dGamma = cController.getGamma();
			Double dLambda = cController.getLambda();

			// // DEBUG - Print the Greeks
			// System.out.println("Greeks!");
			// System.out.println("Alpha - " + dAlpha);
			// System.out.println("Beta - " + dBeta);
			// System.out.println("Kappa - " + dGreek.get(2));
			// System.out.println("Lambda - " + dLambda);
			// System.out.println("Gamma - " + dGamma);

			// Let's get started:
			// Step 1: Generate Test Points
			Vector<Matrix> Chi = new Vector<Matrix>();
			Vector<Matrix> UpChi = new Vector<Matrix>();
			Vector<Matrix> UpY = new Vector<Matrix>();
			Matrix UpPx = new Matrix(3, 3, 0.0);
			Matrix UpPy = new Matrix(3, 3, 0.0);
			Matrix UpPxy = new Matrix(3, 3, 0.0);
			Matrix K;
			Vector<Double> wc = new Vector<Double>();
			Vector<Double> wm = new Vector<Double>();
			Chi.add(X); // Add Chi_0 - the current state estimate (X, Y, Z)

			// Big P Matrix is LxL diagonal
			Matrix SqrtP = SqrtSPKF(P);
			SqrtP = SqrtP.times(dGamma);

			// Set up Sigma Points
			for (int i = 0; i <= 8; i++) {
				Matrix tempVec = SqrtP.getMatrix(0, 8, i, i);
				Matrix tempX = X;
				Matrix tempPlus = tempX.plus(tempVec);
				// System.out.println("TempPlus");
				// tempPlus.print(3, 2);
				Matrix tempMinu = tempX.minus(tempVec);
				// System.out.println("TempMinus");
				// tempMinu.print(3, 2);
				// tempX = X.copy();
				// tempX.setMatrix(i, i, 0, 2, tempPlus);
				Chi.add(tempPlus);
				// tempX = X.copy();
				// tempX.setMatrix(i, i, 0, 2, tempMinu);
				Chi.add(tempMinu);
			}

			// DEBUG Print the lines inside the Chi Matrix (2L x L)
			// for (int i = 0; i<=(2*L); i++){
			// System.out.println("Chi Matrix Set: "+i);
			// Chi.get(i).print(5, 2);
			// }

			// Generate weights
			Double WeightZero = (dLambda / (L + dLambda));
			Double OtherWeight = (1 / (2 * (L + dLambda)));
			Double TotalWeight = WeightZero;
			wm.add(WeightZero);
			wc.add(WeightZero + (1 - (dAlpha * dAlpha) + dBeta));
			for (int i = 1; i <= (2 * L); i++) {
				TotalWeight = TotalWeight + OtherWeight;
				wm.add(OtherWeight);
				wc.add(OtherWeight);
			}
			// Weights MUST BE 1 in total
			for (int i = 0; i <= (2 * L); i++) {
				wm.set(i, wm.get(i) / TotalWeight);
				wc.set(i, wc.get(i) / TotalWeight);
			}

			// //DEBUG Print the weights
			// System.out.println("Total Weight:");
			// System.out.println(TotalWeight);
			// for (int i = 0; i<=(2*L); i++){
			// System.out.println("Weight M for "+i+" Entry");
			// System.out.println(wm.get(i));
			// System.out.println("Weight C for "+i+" Entry");
			// System.out.println(wc.get(i));
			// }

			// Step 2: Propagate Test Points
			// This will also handle computing the mean
			Double ux = dControl.elementAt(0);
			Double uy = dControl.elementAt(1);
			Double uz = dControl.elementAt(2);
			Matrix XhatMean = new Matrix(3, 1, 0.0);
			for (int i = 0; i < Chi.size(); i++) {
				Matrix ChiOne = Chi.get(i);
				Matrix Chixminus = new Matrix(3, 1, 0.0);
				Double Xhat = ChiOne.get(0, 0);
				Double Yhat = ChiOne.get(1, 0);
				Double Zhat = ChiOne.get(2, 0);
				Double Xerr = ChiOne.get(3, 0);
				Double Yerr = ChiOne.get(4, 0);
				Double Zerr = ChiOne.get(5, 0);

				Xhat = Xhat + ux + Xerr;
				Yhat = Yhat + uy + Yerr;
				Zhat = Zhat + uz + Zerr;

				Chixminus.set(0, 0, Xhat);
				Chixminus.set(1, 0, Yhat);
				Chixminus.set(2, 0, Zhat);
				// System.out.println("ChixMinus:");
				// Chixminus.print(3, 2);
				UpChi.add(Chixminus);
				XhatMean = XhatMean.plus(Chixminus.times(wm.get(i)));
			}

			// Mean is right!

			// System.out.println("XhatMean: ");
			// XhatMean.print(3, 2);

			// Step 3: Compute Predicted Mean and Covariance
			// Welp, we already solved the mean - let's do the covariance now
			for (int i = 0; i <= (2 * L); i++) {
				Matrix tempP = UpChi.get(i).minus(XhatMean);
				Matrix tempPw = tempP.times(wc.get(i));
				tempP = tempPw.times(tempP.transpose());
				UpPx = UpPx.plus(tempP);
			}

			// New Steps!

			// Step 4: Compute Measurements! (and Y mean!)
			Matrix YhatMean = new Matrix(3, 1, 0.0);
			for (int i = 0; i <= (2 * L); i++) {
				Matrix ChiOne = Chi.get(i);
				Matrix Chiyminus = new Matrix(3, 1, 0.0);
				Double Xhat = UpChi.get(i).get(0, 0);
				Double Yhat = UpChi.get(i).get(1, 0);
				Double Zhat = UpChi.get(i).get(2, 0);
				Double Xerr = ChiOne.get(6, 0);
				Double Yerr = ChiOne.get(7, 0);
				Double Zerr = ChiOne.get(8, 0);

				Xhat = Xhat + Xerr;
				Yhat = Yhat + Yerr;
				Zhat = Zhat + Zerr;

				Chiyminus.set(0, 0, Xhat);
				Chiyminus.set(1, 0, Yhat);
				Chiyminus.set(2, 0, Zhat);
				UpY.add(Chiyminus);
				YhatMean = YhatMean.plus(Chiyminus.times(wm.get(i)));
			}

			// // Welp, we already solved the mean - let's do the covariances
			// now
			// System.out.println("XHatMean and YHatMean = ");
			// XhatMean.print(3, 2);
			// YhatMean.print(3, 2);

			for (int i = 0; i <= (2 * L); i++) {
				Matrix tempPx = UpChi.get(i).minus(XhatMean);
				Matrix tempPy = UpY.get(i).minus(YhatMean);
				// System.out.println("ChiX - XhatMean and ChiY-YhatMean");
				// tempPx.print(3, 2);
				// tempPy.print(3, 2);

				Matrix tempPxw = tempPx.times(wc.get(i));
				Matrix tempPyw = tempPy.times(wc.get(i));

				tempPx = tempPxw.times(tempPy.transpose());
				tempPy = tempPyw.times(tempPy.transpose());
				UpPy = UpPy.plus(tempPy);
				UpPxy = UpPxy.plus(tempPx);
			}

			// Step 6: Compute Corrections and Update

			// Compute Kalman Gain!
			// System.out.println("Updated Px");
			// UpPx.print(5, 2);
			// System.out.println("Updated Py");
			// UpPy.print(5, 2);
			// System.out.println("Updated Pxy");
			// UpPxy.print(5, 2);
			K = UpPxy.times(UpPy.inverse());
			// System.out.println("Kalman");
			// K.print(5, 2);

			Matrix Mea = new Matrix(3, 1, 0.0);
			Mea.set(0, 0, dMeasure.get(0));
			Mea.set(1, 0, dMeasure.get(1));
			Mea.set(2, 0, dMeasure.get(2));

			Matrix Out = K.times(Mea.minus(YhatMean));
			Out = Out.plus(XhatMean);
			// System.out.println("Out:");
			// Out.print(3, 2);

			Matrix Px = UpPx.minus(K.times(UpPy.times(K.transpose())));

			// Update Stuff!
			// Push the P to the controller
			Matrix OutP = P.copy();
			OutP.setMatrix(0, 2, 0, 2, Px);
			X.setMatrix(0, 2, 0, 0, Out);

			Residual = XhatMean.minus(Out);
			cController.inputState(OutP, Residual);
			// cController.setL(L);

			cController.startProcess();
			while (!cController.finishedProcess()) {
				try {
					Thread.sleep(10);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}

			dGreek.set(0, cController.getAlpha());
			dGreek.set(1, cController.getBeta());
			dGreek.set(2, cController.getKappa());
			P = cController.getP();

			StepDone = true;

		}

	}

	public Vector<GPSEntry> getOutput() {
		if (ProcessDone) {
			return Output;
		}
		return null;
	}

	public Vector<Matrix> getResiduals() {
		if (ProcessDone) {
			return Resid;
		}
		return null;
	}

}
