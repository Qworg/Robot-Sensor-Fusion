package controllers;

import java.util.Vector;
//import javax.vecmath.GMatrix;

import Jama.Matrix;

public interface SPKFControl extends Control{
	Matrix getP();
	Double getAlpha();
	Double getGamma();
	Double getKappa();
	Double getLambda();
	Double getBeta();
	void setL(int L);
	void setAlpha(Double Lam);
	void setKappa(Double Kap);
	void setBeta(Double Bet);
	void inputState(Matrix Cov, Matrix Inno);
	int getControlType();
}
