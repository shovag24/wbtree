//---------------------------------------------------------------------------
#ifndef sa_btreeH
#define sa_btreeH
//---------------------------------------------------------------------------
#include "wbtree.h"
//---------------------------------------------------------------------------
extern float init_avg;
extern float avg_ratio;
extern float lamda;

// Get the optimized Result. It runs till getting a good solution.
double SA_Floorplan(WB_Tree &fp, int k, int local=0, float term_T=0.1);

// Randomly get result. but it is not optimized.
double Random_Floorplan(WB_Tree &fp,int times);
//---------------------------------------------------------------------------
#endif
