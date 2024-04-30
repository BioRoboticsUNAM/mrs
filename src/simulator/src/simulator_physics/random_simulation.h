/************************************************************************
*                                                                       *
*       random_simulaton.h                                              *
*                                                                       *
*       This program generates Uniform or Gaussian random numbers.      *
*                                                                       *
*                                                                       *
*                       J. Savage                                       *
*                       DEPFI-UNAM      11-2003                         *
*                                       10-2022                         *
*                                                                       *
*************************************************************************/
#include <time.h>
#include <math.h>



// generates a uniform random variable
float generaR(float Min,float Max){
 float n;
 //int BIG= 0x7FFFFFFF;
 int BIG= RAND_MAX;

 n=Min+ ((float) random()/(float) BIG ) *(Max-Min);

 return n;
}







/* Function that simulates the success or failure of an action given a probability of failure */
int success(float prb_failure)
{
 int i;
 float rand;
 float tmp;
 static int flg=0;


 if(flg==0){
         // Initializes the random generator
        srandom((int)time((time_t *)NULL));
	flg=1;
 }


 // it generates a Uniform random variable between 0 and 1
 rand=generaR(0.0,1.0);
 printf("prb_failure %f random %f\n",prb_failure,rand);

 tmp=prb_failure;

 if(rand <= tmp){
                i=0;
 } else{
                i=1;
 }
 

 return i;

}


