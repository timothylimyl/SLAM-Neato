#include "mex.h"
#include "math.h"
#include <stdlib.h>

//mex -O scanNE.c

#define PI 3.141592653589793
#define eps  1e-15
#define dot(x,y)   ((x)[0] * (y)[0] + (x)[1] * (y)[1] + (x)[2] * (y)[2])

#define max(a,b) ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a > _b ? _a : _b; })
#define min(a,b) ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a < _b ? _a : _b; })

typedef struct {
    double x0, y0, dx, dy, *x, *y, *z;
    int numX, numY;
} Map;

typedef struct {
    double x0, y0, h0;
    double startDeg, resDeg, maxRange;
    int numScans;
} Scanner;

typedef struct {
    double x, y, h;
} Pose;

//This function returns the intersection of two line segments in 2D
int linSegInt(double x1, double x2, double y1,double y2,double x3, double x4, double y3, double y4, double *xint, double *yint){
    double num1, num2, den, u1, u2;
    int ind1, ind2, ind3;
    
    num1 = (x4-x3)*(y1-y3) - (y4-y3)*(x1-x3);
    num2 = (x2-x1)*(y1-y3) - (y2-y1)*(x1-x3);
    den  = (y4-y3)*(x2-x1) - (x4-x3)*(y2-y1);

    u1      = num1/den;
    u2      = num2/den;

    xint[0] = x1+(x2-x1)*u1;
    yint[0] = y1+(y2-y1)*u1;
    ind1    = (u1 >= 0.0) && (u1 <= 1.0) && (u2 >= 0.0) && (u2 <= 1.0);
    ind2    = ((num1 == 0.0) && (num2 == 0.0) && (ind1 == 1));
    ind3    = (den == 0);
    
    return (ind1==1) && (ind2==0) && (ind3==0);
}

//This function updates the maps
void findRange(Map *map, Scanner *scanner, Pose *pose, double *range, double *xr, double *yr, double *C){
    int L = scanner->numScans;
    double x0, y0, h0, x1, x2, y1 ,y2, xs, ys, xe, ye, r[4], xint[4], yint[4];
    double xmin, xmax, ymin, ymax, ycur;
    int xmini, xmaxi, ymini, ymaxi, ynext;
    int N, M, i, j, hit, ind[4];
    
    double *x = map->x;
    double *y = map->y;
    double *z = map->z;
    
    N = map->numX; //Number of x grid lines (should be num x cells + 1)
    M = map->numY; //Number of y grid lines (should be num y cells + 1)
    
    //Map origin
    double dx    = map->dx;
    double dy    = map->dy;
    double mapx0 = map->x0;
    double mapy0 = map->y0;
    
    x0 = pose->x - scanner->x0;
    y0 = pose->y - scanner->y0;
    h0 = pose->h - scanner->h0;
    
    int xsi, xei, ysi, yei, xci, yci;
    double dxse, dyse, xc, yc, xdist, ydist;
    
    //Loop over the number of scan lines
    for(i=0;i<L;i++){
        
        //Max range to begin with and no hit
        range[i] = scanner->maxRange;
        hit      = 0;
                
        //Calc cos(heading) and sin(heading)
        double cosHead = cos(PI*(h0+scanner->startDeg+i*scanner->resDeg)/180.0);
        double sinHead = sin(PI*(h0+scanner->startDeg+i*scanner->resDeg)/180.0);
        
        //Calc end point of laser ray
        x2 = x0+scanner->maxRange * sinHead;
        y2 = y0+scanner->maxRange * cosHead;
        
        xr[i] = x2;
        yr[i] = y2;
        
        //mexPrintf("x2 = %10.3e, y2 = %10.3e\n",x2,y2);
        
        x1 = x2;
        y1 = y2;
        
        //Clip the line to live inside the main grid box
        xs = x0;
        ys = y0;
        
        //mexPrintf("xs = %10.3e, ys = %10.3e, xe = %10.3e, ye = %10.3e\n",xs,ys,xe,ye);
        //mexPrintf("x[0] = %10.3e, x[N] = %10.3e, y[0] = %10.3e, y[M] = %10.3e\n",x[0],x[N],y[0],y[M]);
        
        if((xs < x[0]) && (fabs(x1-x0) > eps)){
            xs = x[0];
            ys = y0 + (xs-x0)*((y1-y0)/(x1-x0));
        }
        if((ys < y[0]) && (fabs(y1-y0) > eps)){
            ys = y[0];
            xs = x0 + (ys-y0)*((x1-x0)/(y1-y0));
        }
        if((xs > x[N-1]) && (fabs(x1-x0) > eps)){
            xs = x[N-1];
            ys = y0 + (xs-x0)*((y1-y0)/(x1-x0));
        }
        if((ys > y[M-1]) && (fabs(y1-y0) > eps)){
            ys = y[M-1];
            xs = x0 + (ys-y0)*((x1-x0)/(y1-y0));
        }
        
        //mexPrintf("xs = %10.3e, ys = %10.3e, xe = %10.3e, ye = %10.3e\n",xs,ys,xe,ye);
        
        xe = x1;
        ye = y1;
        if((xe < x[0]) && (fabs(x1-x0) > eps)){
            xe = x[0];
            ye = y0 + (xe-x0)*((y1-y0)/(x1-x0));
        }
        if((ye < y[0]) && (fabs(y1-y0) > eps)){
            ye = y[0];
            xe = x0 + (ye-y0)*((x1-x0)/(y1-y0));
        }
        if((xe > x[N-1]) && (fabs(x1-x0) > eps)){
            xe = x[N-1];
            ye = y0 + (xe-x0)*((y1-y0)/(x1-x0));
        }
        if((ye > y[M-1]) && (fabs(y1-y0) > eps)){
            ye = y[M-1];
            xe = x0 + (ye-y0)*((x1-x0)/(y1-y0));
        }
        
        //mexPrintf("xs = %10.3e, ys = %10.3e, xe = %10.3e, ye = %10.3e\n",xs,ys,xe,ye);
        
        
        //Check that the new point is inside the box
        if (   (xs-x[0]>-eps) && (xs-x[N-1]<eps)
            && (ys-y[0]>-eps) && (ys-y[M-1]<eps)
            && (xe-x[0]>-eps) && (xe-x[N-1]<eps)
            && (ye-y[0]>-eps) && (ye-y[M-1]<eps)){
            
            //Determine x and y extremities for laser on this azimuth and polar angles
            //Calc min and max integers for x and y axes
            xsi = floor(max((double)0.0   , (double)((xs-x[0])/dx)));
            xsi = min((double)(N-1) , (double)xsi);
            xei = floor(max((double)0.0   , (double)((xe-x[0])/dx)));
            xei = min((double)(N-1) , (double)xei);
            ysi = floor(max((double)0.0   , (double)((ys-y[0])/dy)));
            ysi = min((double)(M-1) , (double)ysi);
            yei = floor(max((double)0.0   , (double)((ye-y[0])/dy)));
            yei = min((double)(M-1) , (double)yei);
            
            //Set the difference of start to end
            dxse = xe - xs;
            dyse = ye - ys;
            
            //Set the current indices to the starting ones
            xci  = xsi;
            yci  = ysi;
            xc   = xs;
            yc   = ys;
            
            //mexPrintf("xci = %4d, yci = %4d, xs = %10.3e, ys = %10.3e\n",xci,yci,xs,ys);
            
            do{
                //Figure out which way to move next
                if(dxse > 0.0){
                    if(xci<N-1){
                        xdist = (x[xci+1] - xc)/dxse;
                    } else {
                        xdist = 1e20;
                    }
                } else if (dxse < 0.0){
                    if(xci>0){
                        xdist = (x[xci] - xc)/dxse;
                    } else {
                        xdist = 1e20;
                    }
                } else {
                    xdist = 1e20;
                }
                if(dyse > 0.0){
                    if(yci<M-1){
                        ydist = (y[yci+1] - yc)/dyse;
                    } else {
                        ydist = 1e20;
                    }
                } else if (dyse < 0.0){
                    if(yci > 0){
                        ydist = (y[yci] - yc)/dyse;
                    } else {
                        ydist = 1e20;
                    }
                } else {
                    ydist = 1e20;
                }
                
                //mexPrintf("xdist = %10.3e,  ydist = %10.3e, xydiff = %10.3e\n",xdist,ydist,xdist-ydist);
                
                if (fabs(xdist - ydist) < eps){
                    if((dxse > 0.0) && (dyse > 0.0)){
                        xci = min(N-1,xci+1);
                        yci = min(M-1,yci+1);
                        yc  = ys+(x[xci]-xs)*(ye-ys)/dxse;
                        xc  = x[xci];
                    } else if ((dxse < 0.0) && (dyse > 0.0)) {
                        xci = max(0,xci-1);
                        yci = min(M-1,yci+1);
                        yc  = ys+(x[xci]-xs)*(ye-ys)/dxse;
                        xc  = x[xci];
                    } else if ((dxse > 0.0) && (dyse < 0.0)){
                        xci = min(N-1,xci+1);
                        yci = max(0,yci-1);
                        yc  = ys+(x[xci]-xs)*(ye-ys)/dxse;
                        xc  = x[xci];
                    } else if ((dxse < 0.0) && (dyse < 0.0)) {
                        xci = max(0,xci-1);
                        yci = max(0,yci-1);
                        yc  = ys+(x[xci]-xs)*(ye-ys)/dxse;
                        xc  = x[xci];
                    } else {
                        break;
                    }
                } else if(xdist < ydist){
                    if(dxse > 0.0){
                        xci = min(N-1,xci+1);
                        yc  = ys+(x[xci]-xs)*(ye-ys)/dxse;
                        xc  = x[xci];
                    } else if (dxse < 0.0) {
                        xci = max(0,xci-1);
                        yc  = ys+(x[xci]-xs)*(ye-ys)/dxse;
                        xc  = x[xci];
                    } else {
                        break;
                    }
                } else {
                    if(dyse > 0.0){
                        yci = min(M-1,yci+1);
                        xc  = xs+(y[yci]-ys)*(xe-xs)/dyse;
                        yc  = y[yci];
                    } else if (dyse < 0.0) {
                        yci = max(0,yci-1);
                        xc  = xs+(y[yci]-ys)*(xe-xs)/dyse;
                        yc  = y[yci];
                    } else {
                        break;
                    }
                }
                
                //mexPrintf("xci = %4d, yci = %4d\n",xci,yci);
                
                if(z[yci + M*xci]>0){
                    //mexPrintf("* xci = %4d, yci = %4d\n",xci,yci);
                    
                    //Figure out intersection points of ray line with four sides of box
                    ind[0]=linSegInt(x[xci],x[xci]+dx,y[yci]+dy,y[yci]+dy,xs,xe,ys,ye,&xint[0],&yint[0]);
                    ind[1]=linSegInt(x[xci],x[xci]+dx,y[yci],y[yci],xs,xe,ys,ye,&xint[1],&yint[1]);
                    ind[2]=linSegInt(x[xci],x[xci],y[yci],y[yci]+dy,xs,xe,ys,ye,&xint[2],&yint[2]);
                    ind[3]=linSegInt(x[xci]+dx,x[xci]+dx,y[yci],y[yci]+dy,xs,xe,ys,ye,&xint[3],&yint[3]);
                    
                    //Which line did we intersect with
                    int idxInt = 0;
                    
                    //Now figure out which side of the box we intersect with first (i.e. minimum range)
                    for(j=0;j<4;j++){
                        r[j] = sqrt((xint[j]-x0)*(xint[j]-x0) + (yint[j]-y0)*(yint[j]-y0));
                        if ((ind[j] > 0) && (r[j] < range[i])){
                            idxInt   = j;
                            range[i] = r[j];
                            xr[i]    = xint[j];
                            yr[i]    = yint[j];
                        } 
                    }
                    hit = 1;
                    
                    //Horizontal line intersection
                    if ((idxInt==0) || (idxInt==1)){
                        C[i+0*L] = 0.0; //Deriv. w.r.t. x
                        if (sinHead != 0.0){
                            C[i+1*L] = -1.0/sinHead; //Deriv. w.r.t. y
                            C[i+2*L] = -(yr[i]-ys) * cosHead / (sinHead * sinHead); //Deriv. w.r.t. h
                        } else {
                            C[i+1*L] = 0.0; //Deriv. w.r.t. y
                            C[i+2*L] = 0.0; //Deriv. w.r.t. h
                        }
                        
                    //Vertical line intersection
                    } else { 
                        if (cosHead != 0.0){
                            C[i+0*L] = -1.0/cosHead; //Deriv. w.r.t. x
                            C[i+2*L] = (xr[i]-xs) * sinHead / (cosHead * cosHead); //Deriv. w.r.t. h
                        } else {
                            C[i+0*L] = 0.0; //Deriv. w.r.t. x
                            C[i+2*L] = 0.0; //Deriv. w.r.t. h
                        }
                        C[i+1*L] = 0.0; //Deriv. w.r.t. y
                    }
                }
                
            } while (!hit && ((xci != xei) || (yci != yei)));
        }
    }
}


void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    //Setup the map and scanner and pose
    Map     map;
    Scanner scanner;
    Pose    pose;
    
    char *typestr;
    
    mwSize dims[3], numd, *dimd;
    mxArray *tmp;
    
    /* Check that we have the correct number of arguments */
    if(nrhs != 3){
        mexErrMsgTxt("There must be three arguments to this function (map, scanner and pose - see help).");
        return;
    }
    if(nlhs > 3){
        mexErrMsgTxt("Too many output arguments.");
        return;
    }
    
    
    /* Get scanner data */
    tmp = mxGetField(prhs[1], 0, "numScans");
    if (tmp!=NULL){
        scanner.numScans = (int)(mxGetPr(tmp)[0]);
    }
    else {
        mexErrMsgTxt("No such field: scanner.numScans");
        return;
    }
    
    tmp = mxGetField(prhs[1], 0, "startDeg");
    if (tmp!=NULL){
        scanner.startDeg = mxGetPr(tmp)[0];
    }
    else {
        mexErrMsgTxt("No such field: scanner.startDeg");
        return;
    }
    
    tmp = mxGetField(prhs[1], 0, "resDeg");
    if (tmp!=NULL){
        scanner.resDeg = mxGetPr(tmp)[0];
    }
    else {
        mexErrMsgTxt("No such field: scanner.resDeg");
        return;
    }
    
    tmp = mxGetField(prhs[1], 0, "right0");
    if (tmp!=NULL){
        scanner.x0 = mxGetPr(tmp)[0];
    }
    else {
        mexErrMsgTxt("No such field: scanner.right0");
        return;
    }
    
    tmp = mxGetField(prhs[1], 0, "forward0");
    if (tmp!=NULL){
        scanner.y0 = mxGetPr(tmp)[0];
    }
    else {
        mexErrMsgTxt("No such field: scanner.forward0");
        return;
    }
    
    tmp = mxGetField(prhs[1], 0, "angle_down0");
    if (tmp!=NULL){
        scanner.h0 = mxGetPr(tmp)[0];
    }
    else {
        mexErrMsgTxt("No such field: scanner.angle_down0");
        return;
    }
    
    tmp = mxGetField(prhs[1], 0, "maxRange");
    if (tmp!=NULL){
        scanner.maxRange = mxGetPr(tmp)[0];
    }
    else {
        mexErrMsgTxt("No such field: scanner.maxRange");
        return;
    }
    
    //Get map data
    tmp = mxGetField(prhs[0], 0, "east0");
    if (tmp!=NULL){
        map.x0 = mxGetPr(tmp)[0];
    }
    else {
        mexErrMsgTxt("No such field: map.east0");
        return;
    }
    
    tmp = mxGetField(prhs[0], 0, "north0");
    if (tmp!=NULL){
        map.y0 = mxGetPr(tmp)[0];
    }
    else {
        mexErrMsgTxt("No such field: map.north0");
        return;
    }
    
    tmp = mxGetField(prhs[0], 0, "Deast");
    if (tmp!=NULL){
        map.dx = mxGetPr(tmp)[0];
    }
    else {
        mexErrMsgTxt("No such field: map.Deast");
        return;
    }
    
    tmp = mxGetField(prhs[0], 0, "Dnorth");
    if (tmp!=NULL){
        map.dy = mxGetPr(tmp)[0];
    }
    else {
        mexErrMsgTxt("No such field: map.Dnorth");
        return;
    }
    
    tmp = mxGetField(prhs[0], 0, "east");
    if (tmp!=NULL){
        map.x = mxGetPr(tmp);
    }
    else {
        mexErrMsgTxt("No such field: map.east");
        return;
    }
    
    tmp = mxGetField(prhs[0], 0, "north");
    if (tmp!=NULL){
        map.y = mxGetPr(tmp);
    }
    else {
        mexErrMsgTxt("No such field: map.north");
        return;
    }
    
    tmp = mxGetField(prhs[0], 0, "Z");
    if (tmp!=NULL){
        map.z = mxGetPr(tmp);
    }
    else {
        mexErrMsgTxt("No such field: map.Z");
        return;
    }
    
    tmp = mxGetField(prhs[0], 0, "Neast");
    if (tmp!=NULL){
        map.numX = (int)(mxGetPr(tmp)[0]);
    }
    else {
        mexErrMsgTxt("No such field: map.Neast");
        return;
    }
    
    tmp = mxGetField(prhs[0], 0, "Nnorth");
    if (tmp!=NULL){
        map.numY = (int)(mxGetPr(tmp)[0]);
    }
    else {
        mexErrMsgTxt("No such field: map.Nnorth");
        return;
    }
    
    //Now the pose information
    tmp = mxGetField(prhs[2], 0, "east");
    if (tmp!=NULL){
        pose.x = mxGetPr(tmp)[0];
    }
    else {
        mexErrMsgTxt("No such field: pose.east");
        return;
    }
    
    tmp = mxGetField(prhs[2], 0, "north");
    if (tmp!=NULL){
        pose.y = mxGetPr(tmp)[0];
    }
    else {
        mexErrMsgTxt("No such field: pose.north");
        return;
    }
    
    tmp = mxGetField(prhs[2], 0, "psi");
    if (tmp!=NULL){
        pose.h = mxGetPr(tmp)[0];
    }
    else {
        mexErrMsgTxt("No such field: pose.psi");
        return;
    }
    
    
    plhs[0] = mxCreateDoubleMatrix(scanner.numScans,1,mxREAL);
    double *range = mxGetPr(plhs[0]);
    
    plhs[1] = mxCreateDoubleMatrix(scanner.numScans,1,mxREAL);
    double *xr = mxGetPr(plhs[1]);
    
    plhs[2] = mxCreateDoubleMatrix(scanner.numScans,1,mxREAL);
    double *yr = mxGetPr(plhs[2]);
        
    plhs[3] = mxCreateDoubleMatrix(scanner.numScans,5,mxREAL);
    double *C = mxGetPr(plhs[3]);
    
    //Now loop over the scans and find the range
    findRange(&map,&scanner,&pose,range,xr,yr,C);
}