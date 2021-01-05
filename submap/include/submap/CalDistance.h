// Wasserstein Distance
// https://en.wikipedia.org/wiki/Wasserstein_metric
// https://zhuanlan.zhihu.com/p/58506295


#include <iostream>
#include "math.h"
double CalDistance(int dim,double* mean1, double*var1, double* mean2, double*var2){
    double distance;
    distance=pow((mean1-mean2),2)+trace(dim, Madd(dim, var1,var2,MM(dim, var1,var2)));
    return distance;
}

double trace(int dim, double* var){
    float trace=0.0;
    for (int i=0; i<dim; i++){
        trace=trace+var[i];
    }
    return trace;
}


//Matrix multi Matrix for Wasserstein Distance
double* MM(int dim, double* var1, double* var2){
    double* result= var1;
    for (int i=0; i<dim; i++){
        result[i]=sqrt(var2[i])*result[i]*sqrt(var2[i]);
        result[i]=2*sqrt(result[i]);
    }
    return result;
}

//Matrix addMatrix for Wasserstein Distance
double* Madd(int dim, double* var1, double* var2, double* var3){
double* result= var1;
    for (int i=0; i<dim; i++){
        result[i]=result[i]+var2[i];
        result[i]=result[i]+var3[i];
    }
    return result;
}