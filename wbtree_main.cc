// Project: B*-tree based placement/floorplanning
// Advisor: Yao-Wen Chang  <ywchang@cis.nctu.edu.tw>
// Authors: Jer-Ming Hsu   <barz@cis.nctu.edu.tw>
// 	    Hsun-Cheng Lee <gis88526@cis.nctu.edu.tw>
// Sponsors: NSC, Taiwan; Arcadia, Inc.; UMC.
// Version 1.0
// Date:    7/19/2000

//---------------------------------------------------------------------------
#include <iostream>
#include <cstring>
#include "wbtree.h"
#include "sa.h"
//---------------------------------------------------------------------------

int main(int argc,char **argv)
{
  char filename[80] = "apte",outfile[80]="";
  int times=15, local=7;
  float init_temp=0.9, term_temp=0.1;
  float alpha=1;
  //srand(time(0));
 if(argc<=1){
   printf("Usage: btree <filename> [times=%d] [hill_climb_stage=%d]\n",
         times, local);
   printf("        [avg_ratio=%.1f] [cost_ratio=%f]\n",avg_ratio,alpha);
   printf("        [lamda=%.2f] [term-temp=%.2f]\n",lamda,term_temp);
   printf("        [output]\n");
   return 0;
 }else{
   int argi=1;
   if(argi < argc) strcpy(filename, argv[argi++]);
   if(argi < argc) times=atoi(argv[argi++]);
   if(argi < argc) local=atoi(argv[argi++]);
   if(argi < argc) avg_ratio=atof(argv[argi++]);
   if(argi < argc) alpha=atof(argv[argi++]);
   if(argi < argc) lamda=atof(argv[argi++]);
   if(argi < argc) term_temp=atof(argv[argi++]);
   if(argi < argc) strcpy(outfile, argv[argi++]);
 }
  
   try{
     double time = seconds();
     WB_Tree wbt;
     wbt.init(alpha, filename, times, local, term_temp);

     double last_time =  SA_Floorplan(wbt, times, local, term_temp);
     wbt.show_tree();
     //Random_Floorplan(fp,times);
     wbt.list_information();

     { // log performance and quality
       if(strlen(outfile)==0)
         strcpy(outfile,strcat(filename,".res"));

       last_time = last_time - time;
       printf("CPU time       = %.2f\n",seconds()-time);
       printf("Last CPU time  = %.2f\n",last_time);
       FILE *fs= fopen(outfile,"a+");


       fprintf(fs,"CPU= %.2f, Cost= %.6f, Area= %.0f, Wire= %.0f, Dead=%.4f ",
               last_time, float(wbt.cost),  float(wbt.Area),
                float(wbt.WireLength), float(wbt.get_dead_space()));
       fprintf(fs," :%d %d %.0f \n", times, local, avg_ratio);
       fclose(fs);
     }
     wbt.output();

   }catch(...){}
   return 1;
}
