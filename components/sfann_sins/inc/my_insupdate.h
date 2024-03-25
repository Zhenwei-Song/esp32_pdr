/*
 * File: my_insupdate.h
 */
#ifndef MY_INSUPDATE_H
#define MY_INSUPDATE_H
#ifdef __cplusplus
extern "C" {
#endif
#include "MyMatrix.h"

Matrix my_insupdate(Matrix qnb, Matrix vn1, Matrix pos, Matrix wm, Matrix vm, double nts);
#ifdef __cplusplus
}
#endif
#endif
