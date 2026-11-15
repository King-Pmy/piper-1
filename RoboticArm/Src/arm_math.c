#include "arm_math.h"

void Matrix_Multiply4x4(Matrix4x4 *A,Matrix4x4 *B,Matrix4x4 *C){
    Matrix4x4 T;
    for(int i= 0;i<4;i++){
        for(int j =0;j<4;j++){
            float sum = 0.0f;
            for(int k =0;k<4;k++){
                sum += A->data[i][k]*B->data[k][j];
                T.data[i][j] = sum;
            }
        }
    }
    *C = T;
}

void Matrix_Indentity4x4(Matrix4x4 *mat){
    for(int i = 0;i<4;i++){
        for(int j = 0;j<4;j++){
            mat->data[i][j] = (i == j)? 1.0f : 0.0f;
        }
    }
}