#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <Arduino.h>
#include "mag_cal.h"
#define XtX_RANK 4

/*
Updates the XtX matrix iteratively. 
xtx is the matrix to be updated
Bp is the magnetometer reading array
*/
int updateXtX(T_MATRIX xtx[][XtX_RANK], T_MATRIX* Bp){
	for (int i = 0; i < XtX_RANK; ++i)
	{
		for (int j = 0; j < XtX_RANK; ++j)
		{
			xtx[i][j] += Bp[i]*Bp[j];
		}
	}
	
	return 0;
}
/*
Updates the XtY matrix iteratively. 
xty is the matrix to be updated
Bp is the magnetometer reading array
*/
int updateXtY(T_MATRIX* xty, T_MATRIX* Bp){
	for (int i = 0; i < XtX_RANK; ++i)
		 xty[i] += Bp[i]*(Bp[0]*Bp[0] + Bp[1]*Bp[1] + Bp[2]*Bp[2]);
	return 0;
}
void printMatrix(T_MATRIX matrix[][XtX_RANK], int rows, int cols) {
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            DEBUG_PRINT(matrix[i][j],4);
            DEBUG_PRINT("\t");
        }
        DEBUG_PRINTLN();
    }
}

void printVector(T_MATRIX* vector, int rows) {
    for (int i = 0; i < rows; i++)
            DEBUG_PRINTLN(vector[i],4);
}


void InvertMatrix(T_MATRIX A[][XtX_RANK],T_MATRIX B[][XtX_RANK],int n)
{
	int i,j,k;
	T_MATRIX tmp=0.0;
	for(i=0;i<n;i++){
		for(j=0;j<n;j++){
			if(i==j){
				B[i][j]=1.0;
			}
			else{
				B[i][j]=0.0;
			}
		}
	}
	if(n==1) B[0][0]=(1.0)/A[0][0];
	else{
		for(j=0;j<n;j++){
			for(i=n-1;i>0;i--){
				if(i>j){
					tmp=A[i][j]/A[i-1][j];
					for(k=0;k<n;k++){
						A[i][k]-=tmp*A[i-1][k];
						B[i][k]-=tmp*B[i-1][k];
					}
				}
			}
		}
		for(j=n-1;j>0;j--){
			for(i=0;i<n;i++){
				if(i<j){
					tmp=A[i][j]/A[i+1][j];
					for(k=0;k<n;k++){
						A[i][k]-=tmp*A[i+1][k];
						B[i][k]-=tmp*B[i+1][k];
					}
				}
			}
		}
	}
	for(i=0;i<n;i++){
		tmp=A[i][i];
		for(j=0;j<n;j++){
			A[i][j]/=tmp;
			B[i][j]/=tmp;
		}
	}
}

void mxbyvector(T_MATRIX x[][XtX_RANK],T_MATRIX *v, T_MATRIX *y,int n)
{
        int i,j;
        T_MATRIX tmp=0.0;
        for(i=0;i<n;i++){
                tmp=0.0;
                for(j=0;j<n;j++){
                        tmp+=x[i][j]*v[j];
                }
                y[i]=tmp;
        }
}



T_MATRIX XtX[XtX_RANK][XtX_RANK];
T_MATRIX XtXinv[XtX_RANK][XtX_RANK];
T_MATRIX XtY[XtX_RANK];
T_MATRIX beta[XtX_RANK];
T_MATRIX V[XtX_RANK];
T_MATRIX B;
int calibrated=0;

int MagSensorInitCalibration(){
	int i,j;
 	calibrated=0;

	for(i=0;i<XtX_RANK;i++) for(j=0;j<XtX_RANK;j++){
		XtX[i][j]=0;	
		XtXinv[i][j]=0;
	}
	for(i=0;i<XtX_RANK;i++){
		XtY[i]=0;
		beta[i]=0;
	} 
	return 0;
}


int MagSensorUpdateCalibration(T_MATRIX *Bp){

	int r = updateXtX(XtX, Bp);
	r += updateXtY(XtY, Bp);
  calibrated=0;
  return r; //0 on success
}
int MagSensorCalculateCalibrationResult(void){
  	DEBUG_PRINT("\nXtX=\n");
		printMatrix(XtX,XtX_RANK,XtX_RANK);

	InvertMatrix(XtX,XtXinv,XtX_RANK);
	mxbyvector(XtXinv,XtY,beta,XtX_RANK);
	
	for (int i = 0; i < XtX_RANK-1; ++i)
	{
		V[i] = 0.5*beta[i];
	}
	V[3] = sqrt(beta[3] + V[0]*V[0] + V[1]*V[1] + V[2]*V[2]);
  	DEBUG_PRINT("\nXtY=\n");
		printVector(XtY,XtX_RANK);

		DEBUG_PRINT("\ninv(XtX)=\n");
		printMatrix(XtXinv,XtX_RANK,XtX_RANK);

		DEBUG_PRINT("\nBeta=inv(XtX)*XtY=\n");
		printVector(beta,XtX_RANK);

		DEBUG_PRINT("\nV=\n");
		printVector(V,3);

		DEBUG_PRINT("\nB=");
    DEBUG_PRINTLN(B);
	
 	calibrated=1;

	return 0;
}

/*
Writes the hard iron magnetic calibration vector to V_hard_iron_offset
Returns the number of items written
*/
int MagSensorGetCalibrationResult(T_MATRIX *V_hard_iron_offset){
	if(calibrated){
		for (int i = 0; i < XtX_RANK-1; ++i)
			V_hard_iron_offset[i] = V[i];
		return 3;
	}
	else
		return 0;
}

T_MATRIX MagSensorGetMagneticFieldStrength_uT(){
	if(calibrated)
		return V[3];
	else
		return 0;
}




	
