package controllers;

import java.util.Vector;

//Import the Fuzzy Logic system
import net.sourceforge.jFuzzyLogic.FIS;
import net.sourceforge.jFuzzyLogic.rule.FuzzyRuleSet;

import Jama.Matrix;

/**
 * This is the Fuzzy Controller for the SPKF. It includes both the single and
 * double fuzzy control methods.
 * 
 * @category Controller
 * @author jeffreykramer
 * 
 */
public class FuzzySPKF implements SPKFControl {
	FIS fisA = null;

	FIS fisB = null;
	
	FIS fisC = null;

	FuzzyRuleSet frsA = null;

	FuzzyRuleSet frsB = null;
	
	FuzzyRuleSet frsC = null;

	private Matrix P;

	private Matrix Innov;
	
	private Matrix Deviation;

	private Double Alpha;

	private Double Beta;

	private Double Kappa;

	private int L;

	private Boolean Started = false;

	private Boolean Done = false;

	private Boolean DFK;

	private Boolean bGPS;

	/**
	 * Single Fuzzy Constructor
	 * 
	 * @param FN
	 * @param iL
	 */
	public FuzzySPKF(String FN, int iL) {
		super();
		fisA = FIS.load(FN);
		frsA = fisA.getFuzzyRuleSet();
		L = iL;
		DFK = false;
		bGPS = false;
	}

	/**
	 * Double Fuzzy Constructor
	 * 
	 * @param FNA
	 * @param FNB
	 * @param iL
	 */
	public FuzzySPKF(String FNA, String FNB, int iL) {
		super();
		fisA = FIS.load(FNA);
		frsA = fisA.getFuzzyRuleSet();
		fisB = FIS.load(FNB);
		frsB = fisB.getFuzzyRuleSet();
		fisC = FIS.load("newdfkfis.txt");
		frsC = fisC.getFuzzyRuleSet();
		L = iL;
		DFK = true;
		bGPS = false;
	}

	public Matrix getP() {
		// Return the Matrix P... but not unless we're done with it!
		if (Done && Started) {
			Done = false;
			Started = false;
			return P;
		} else {
			System.out.println("Not done with P!  You can't have it!");
		}
		return null;
	}

	public Boolean finishedProcess() {
		return Done;
	}

	public void inputState(Vector<Vector<Double>> State) {
		System.out.println("In the interests of time, we don't support vector vector madness here.  Try again later.");
	}

	public void inputState(Matrix State) {
		System.out.println("We don't do this here... =0");
	}

	public void inputState(Matrix Cov, Matrix Inno) {
		// 'ere we go!
		P = Cov;
		Innov = Inno;
	}

	public void isGPS(Matrix oIMU) {
		bGPS = true;
		Deviation = oIMU;
	}

	public void startProcess() {
		if (!Started) {
			Started = true;
			// P has Px and our Covariances in it
			// Innov is our innovations
			Double InnoX = Innov.get(0, 0);
			Double InnoY = Innov.get(1, 0);
			Double InnoZ = Innov.get(2, 0);
			Double PX = P.get(0, 0);
			Double PY = P.get(1, 1);
			Double PZ = P.get(2, 2);
			Double dQx = P.get(3, 3);
			Double dQy = P.get(4, 4);
			Double dQz = P.get(5, 5);
			Double dRx = P.get(6, 6);
			Double dRy = P.get(7, 7);
			Double dRz = P.get(8, 8);
			Double alphax, alphay, alphaz;

			// System.out.println("In Fuzzy SPKF!");

			if (DFK) {
//				 System.out.println("In Double Fuzzy SPKF!");

				// TWO METHODS:
				// 1 - Greek Modification
				// 2 - Fusion vs. IMU - per step error vs. sensor response

				if (!bGPS) {
//					System.out.println("In Old Double");
					// So, in here we'll be editing our greeks
					// alpha must 0 <= a <= 1, with tending towards zero
					// beta must be >0 with 2 representing a Gaussian system
					// kappa must be greater than zero, with no cap (unknown
					// providence)
					Double falpha, fbeta, fkappa;

					// This doesn't work on individual covariances, so we'll
					// average
					// our covariance, and our innovations (which should work,
					// but
					// must be tested)
					// TODO Make sure that averaging these works appropriately

					Double InnoAvg = (InnoX + InnoY + InnoZ) / 3;
					Double PAvg = (PX + PY + PZ) / 3;
					Double RAvg = (dRx + dRy + dRz) / 3;

					InnoAvg = Math.abs(InnoAvg);

					// System.out.println("avg: " + InnoAvg);
					// System.out.println("covar: "+PAvg/RAvg);

					frsB.setVariable("covar", Math.abs(PAvg / RAvg));
					frsB.setVariable("avg", InnoAvg);
					frsB.evaluate();
					falpha = frsB.getVariable("alphamod").defuzzify();
					fbeta = frsB.getVariable("betamod").defuzzify();
					// fkappa = frsB.getVariable("kappamod").defuzzify();

					// System.out.println("Fuzzy Mults: " + falpha + " , " +
					// fbeta);

					// Check for Sanity!
					falpha = falpha * Alpha;
					if (falpha > 0.4)
						falpha = 0.4;
					if (falpha < 0.000001)
						falpha = 0.000001;
					fbeta = fbeta * Beta;
					if (fbeta > 4)
						fbeta = 4.0;
					if (fbeta <= 0.5)
						fbeta = 0.5;

					this.setAlpha(falpha);
					this.setBeta(fbeta);
					// this.setKappa(fkappa * Kappa);

				}else{
					//This is the "new method" of DFSPKF - checking per step error of IMU vs. Sensor Response
//					System.out.println("In New DFSPKF");
					double TrueDev = Math.sqrt(Deviation.get(0, 0)*Deviation.get(0, 0)+Deviation.get(1, 0)*Deviation.get(1, 0));
//					System.out.println("TrueDev ="+TrueDev);
					frsC.setVariable("dev", TrueDev);
					frsC.evaluate();
					double gmod = frsC.getVariable("gmod").defuzzify();
//					System.out.println("gmod: " + gmod);
					dRx = dRx * gmod;
					dRy = dRy * gmod;
					dRz = dRz * gmod;
//					System.out.println("dR = "+dRx+","+dRy+","+dRz);
				}
			}

			// Fuzzy Work
			// X
			frsA.setVariable("covar", (PX / dRx));
			frsA.setVariable("avg", InnoX);
			frsA.evaluate();
			alphax = frsA.getVariable("alpha").defuzzify();
			// Y
			frsA.setVariable("covar", (PY / dRy));
			frsA.setVariable("avg", InnoY);
			frsA.evaluate();
			alphay = frsA.getVariable("alpha").defuzzify();
			// Z
			frsA.setVariable("covar", (PZ / dRz));
			frsA.setVariable("avg", InnoZ);
			frsA.evaluate();
			alphaz = frsA.getVariable("alpha").defuzzify();

			// System.out.println("dQ, dR Mults: "+ alphax + "," + alphay + ","
			// + alphaz);

			// Update R/Q
			dQx = dQx * alphax;
			dQy = dQy * alphay;
			dQz = dQz * alphaz;
			if (!bGPS || !DFK) {
				dRx = dRx * alphax;
				dRy = dRy * alphay;
				dRz = dRz * alphaz;
			}

//			System.out.println("dQ: " + dQx + ","+ dQy + ","+ dQz);
//			System.out.println("dR: " + dRx + ","+ dRy + ","+ dRz);

			// Sanity
			if (dQx > 100.0)
				dQx = 100.0;
			if (dQx < 0.001)
				dQx = 0.001;
			if (dQy > 100.0)
				dQy = 100.0;
			if (dQy < 0.001)
				dQy = 0.001;
			if (dQz > 100.0)
				dQz = 100.0;
			if (dQz < 0.001)
				dQz = 0.001;
			if (dRx > 100.0)
				dRx = 100.0;
			if (dRx < 0.001)
				dRx = 0.001;
			if (dRy > 100.0)
				dRy = 100.0;
			if (dRy < 0.001)
				dRy = 0.001;
			if (dRz > 100.0)
				dRz = 100.0;
			if (dRz < 0.001)
				dRz = 0.001;

			P.set(3, 3, dQx);
			P.set(4, 4, dQy);
			P.set(5, 5, dQz);
			P.set(6, 6, dRx);
			P.set(7, 7, dRy);
			P.set(8, 8, dRz);

			Done = true;
			bGPS = false;
			// Started = false;
		}
	}

	public Double getAlpha() {
		return Alpha;

	}

	public Double getBeta() {
		return Beta;
	}

	public Double getGamma() {
		return Math.sqrt(L + this.getLambda());
	}

	public Double getKappa() {
		return Kappa;
	}

	public Double getLambda() {
		return ((Alpha * Alpha) * (L + Kappa)) - L;
	}

	public void setL(int iL) {
		System.out.println("This is likely a bad idea... You're changing L!");
		L = iL;
	}

	public void setAlpha(Double Alp) {
		Alpha = Alp;
	}

	public void setBeta(Double Bet) {
		Beta = Bet;
	}

	public void setKappa(Double Kap) {
		Kappa = Kap;
	}

	public int getControlType() {
		// TODO Auto-generated method stub
		return 2;
	}

}
