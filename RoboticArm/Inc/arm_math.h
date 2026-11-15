#ifndef __ARM_MATH_H
#define __ARM_MATH_H

typedef struct{
    float data[4][4];
}Matrix4x4;

void Matrix_Multiply4x4(Matrix4x4 *A,Matrix4x4 *B,Matrix4x4 *C);
void Matrix_Indentity4x4(Matrix4x4 *mat);

#endif/*之后或可用CMSIS-DSP库*/
