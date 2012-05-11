package controllers;

import java.util.Vector;

import Jama.Matrix;

public class NoControlSPKFTest implements SPKFControl {

	private Matrix P;

	private Matrix Innov;

	private Double Alpha;

	private Double Beta;

	private Double Kappa;

	private int L;

	private Boolean Started = false;

	private Boolean Done = false;

	public NoControlSPKFTest(int iL) {
		L = iL;
	}

	public Matrix getP() {
		// Return the Matrix P... but not unless we're done with it!
		if (Done) {
			Done = false;
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

	public void startProcess() {
		Started = true;
		//We would screw with the Q and R in here.
		
		Done = true;
		Started = false;
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
		return (((Alpha * Alpha) * (L + Kappa)) - L);
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
		return 1;
	}

}
