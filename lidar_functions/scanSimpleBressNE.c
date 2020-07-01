#include "mex.h"
#include "math.h"
#include <stdlib.h>

//mex -O scanSimpleBressNE.c

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

//This function updates the maps
void findRangeNew(Map *map, Scanner *scanner, Pose *pose, double *range, double *xr, double *yr){
    int L = scanner->numScans;
    float h0, xs, ys, xe, ye;
    int N, M, i, j, cnt;
    
    int width  = map->numX; //Number of x grid lines (should be num x cells + 1)
    int height = map->numY; //Number of y grid lines (should be num y cells + 1)
    
    //Map origin
    float dx    = map->dx;
    float dy    = map->dy;
    float mapx0 = map->x0;
    float mapy0 = map->y0;
    
    xs = pose->x - scanner->x0;
    ys = pose->y - scanner->y0;
    h0 = pose->h - scanner->h0;
    
    int   xc, yc, xstep, ystep, hit, x0, y0, x1, y1, x2, y2;
    float deltax, deltay, err, derr, xerr, yerr;
    
    //mexPrintf("x0 = %10.3e, y0 = %10.3e\n",x0,y0);
    
    //Loop over the number of scan lines
    for(i=0;i<L;i++){
        //Calc end point of laser ray
        xe = xs+scanner->maxRange*sin(PI*(h0+scanner->startDeg+i*scanner->resDeg)/180.0);
        ye = ys+scanner->maxRange*cos(PI*(h0+scanner->startDeg+i*scanner->resDeg)/180.0);
        
        range[i] = scanner->maxRange;
        xr[i]    = xe;
        yr[i]    = ye;
        
        x1 = round(xe/dx);
        y1 = round(ye/dy);
        
        x0 = round(xs/dx);
        y0 = round(ys/dy);
        
    	int steep = 0;
        if (abs(y1-y0) > abs(x1-x0)){ 
            steep = 1;
        }

        if (steep) {
            x2 = x0;
            x0 = y0;
            y0 = x2;
            x2 = x1;
            x1 = y1;
            y1 = x2;
        }

        hit = 0;
        cnt = 0;
        
        if (x0 != x1){
            if (steep){
                deltax = fabs(ye-ys)/dy;
                deltay = fabs(xe-xs)/dx;
            } else {
                deltax = fabs(xe-xs)/dx;
                deltay = fabs(ye-ys)/dy;
            }

            err  = 0;
            derr = deltay;
            xc = x0;
            yc = y0;

            xstep = -1;
            if (x0 < x1) xstep = 1;

            ystep = -1;
            if (y0 < y1) ystep = 1;
            
            while ((cnt < width+height) && (hit == 0) && (xc != x1)) {                
                cnt++;
                if (!steep) {
                    if (0 <= yc && yc < height && 0 <= xc && xc < width && map->z[yc + height*xc]>0){
                        xr[i]    = map->x[xc] + dx/2;
                        yr[i]    = map->y[yc] + dy/2;
                        xerr     = xr[i] - xs;
                        yerr     = yr[i] - ys;
                        range[i] = sqrt(xerr*xerr + yerr*yerr);
                        hit      = 1;
                    }
                } else { 
                    if (0 <= xc && xc < height && 0 <= yc && yc < width && map->z[xc + height*yc]>0){
                        xr[i]    = map->x[yc] + dx/2;
                        yr[i]    = map->y[xc] + dy/2;
                        xerr     = xr[i] - xs;
                        yerr     = yr[i] - ys;
                        range[i] = sqrt(xerr*xerr + yerr*yerr);
                        hit      = 1;
                    }
                }

                xc  += xstep;
                err += derr;

                if (err * 2.00 >= deltax) {
                    yc  += ystep;
                    err -= deltax;
                }

                //mexPrintf("xc = %10.3e, yc = %10.3e\n",xc,yc);            
            }
            if (cnt >= width+height){
                mexPrintf("i = %4d, steep = %4d, xstep = %4d, ystep = %4d, x1 = %10.3e, y1 = %10.3e, x0 = %10.3e, y0 = %10.3e, xc = %10.3e, yc = %10.3e\n",i,steep,xstep,ystep,x1,y1,x0,y0,xc,yc);
            }
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
        
    //Now loop over the scans and find the range
    findRangeNew(&map,&scanner,&pose,range,xr,yr);
}