#include<stdio.h> 
#include<math.h>

main()

{

float a,b,c,ca,cb,cc;

scanf("%f %f %f",&a,&b,&c);

ca=(b*b+c*c-a*a)/(2*c*b);

cb=(a*a-b*b+c*c)/(2*c*a);

cc=(a*a+b*b-c*c)/(2*a*b);

printf("<a=%f\n",acos(ca)*180/3.1415926);

printf("<b=%f\n",acos(cb)*180/3.1415926);

printf("<c=%f\n",acos(cc)*180/3.1415926);

    //getch();

 }
