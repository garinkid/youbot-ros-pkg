/**********************************************************
 * This file is generated by 20-sim ANSI-C Code Generator  
 *
 *  file:  common\xxinverse.h
 *  subm:  PositionInteractionControl
 *  model: ControlYoubotArmInteraction
 *  expmt: ControlYoubotArmInteraction
 *  date:  April 8, 2011
 *  time:  5:53:21 pm
 *  user:  Campuslicentie 
 *  from:  Universiteit Twente
 *  build: 4.1.2.2
 **********************************************************/

#ifndef XX_INVERSE_H
#define XX_INVERSE_H

/* 20-sim include files */
#include "xxtypes.h"
#include "xxmatrix.h"

/* 20-sim function prototypes */
void XXIndex (XXMatrix *v);
void XXPermute (XXMatrix *v, XXMatrix *p, XXDouble *workarray);
void XXSubstitute (XXMatrix *dest, XXMatrix *v);
void XXSwapRows (XXMatrix *dest, XXInteger row1, XXInteger row2);
XXInteger XXPivot (XXMatrix *dest, XXMatrix *p, XXInteger i);
XXDouble XXDecompose (XXMatrix *dest, XXMatrix *p);
XXDouble XXCrout1 (XXMatrix *dest, XXMatrix *v, XXDouble *workarray);
XXDouble XXCrout2 (XXMatrix *dest, XXMatrix *CroutMat, XXMatrix *x, XXMatrix *y, XXDouble *workarray);
XXDouble XXInverse (XXMatrix *mat_dest, XXMatrix *mat_source, XXDouble *workarray);
//XXDouble XXMatrixDeterminant (XXMatrix *mat_source);
//void XXMatrixInverse (XXMatrix *mat_dest, XXMatrix *mat_source, XXDouble *workarray);
//void XXMatrixDiv (XXMatrix *mat_dest, XXMatrix *mat_source1, XXMatrix *mat_source2, XXDouble *workarray);
//void XXMatrixScalarDiv (XXMatrix *mat_dest, XXMatrix *mat_source1, XXDouble s2);
//void XXScalarMatrixDiv (XXMatrix *mat_dest, XXDouble *s1, XXMatrix *mat_source2, XXDouble *workarray);

#endif

